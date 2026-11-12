#!/bin/sh
# A starter robot for rules v2, on the Clunkyard shell SDK (clunkyard.jq,
# next to this file). It sweeps a medium scan to find a robot, or points it
# where a passive contact was heard, narrows the scan to hold it, leads it
# with shots whose power suits the range, fires only when the turret is on
# target, keeps battery back for driving and keeps out of the storm.
#
# Starting a program every turn would cost more CPU time than a match
# allows, so the shell hands the match to one jq, and the SDK is a jq
# module: the game is the hello, the state is the state message, and the
# turn returns the act and a memory that comes back next turn.
#
# The trade-offs ($game has the numbers from hello):
#   scan     a narrow arc reaches further; a wide one finds robots nearby
#   power    heavy shots hit harder for less battery, but fly slowly
#   battery  shots and driving share it, and it refills slowly
#   turret   it turns at a limited rate; $me.aim is where it points now
#   cues     passive contacts (a ping, a scan beam, a shot heard) say which
#            way to look, not where to shoot: they have no id and a noisy
#            bearing, so cy::nearest leaves them out
# Clunkyard robot protocol 2: one JSON object per line on stdin (hello, then
# a state each tick) and stdout (ready, then one act per state). Write
# debug output to stderr.
# hello: self {id name} match {mode ruleset control tickRate maxTicks
#   robots} arena {width height depth gravity terrain} robot {radius
#   maxHealth maxSpeed maxTurnRate maxPitchTurnRate maxPitch mass
#   momentOfInertia maxThrust maxTorque maxAngularSpeed brakeDeceleration
#   brakeAngularDeceleration} battery {capacity regen drivePerSecond
#   strafeFactor compute} gun {turnRate power damage escalation shotSpeed
#   reloadTicks cost spread shotRadius shotRange muzzleDistance}
#   scanner {area minRange maxRange minArc maxArc startArc proximityRange}
#   suddenDeath {startTick dps radiusStart radiusEnd shape center endTick}
#   sensing {faintFactor beam muzzle motionSpeed pingTicks range
#   bearingErrorPer100 rangeError rangeBin} objectives {score pickups
#   hill} storm {radius startTick speed growth dps} derived {hoverLift
#   sustainableDrive}
# state: tick self {health battery position velocity heading pitch roll
#   angularVelocity attitude aim cooldownTicks spread grounded altitude
#   groundHeight applied} sensors {scan contacts} zone objectives {score
#   pickups armour hill} storm derived {self gun zone storm objectives}
# act, every part optional: tick drive {throttle turn pitchTurn}
#   forces {thrust strafe lift yawTorque pitchTorque rollTorque brake}
#   aim {heading pitch} scan {heading pitch arc} fire {power} status {text
#   value}
# ready: sdk avatar
# Angles are degrees from the +x axis; pitch is positive up. bot.json
# carries "protocol": 2.
# Manual: https://clunkyard.com/manual#protocol

# The jq program's $names are jq variables, not shell expansions.
# shellcheck disable=SC2016
exec jq -nc --unbuffered -L "$(dirname "$0")" '
import "clunkyard" as cy;

def search_turn: 0.33;  # enough to turn against the brake in the force rulesets

# The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit
# xorshift and a square-root sum. jq has no bitwise operators, so x is
# sixteen 4-bit digits, least significant first, and XOR looks digits up
# in a table.
def calibration_work($work; $tick):
  [range(16) as $a | range(16) as $b | reduce (8, 4, 2, 1) as $p (0; . + ((($a / $p | floor) + ($b / $p | floor)) % 2) * $p)] as $t
  | def xor($y): . as $x | [range(16) as $i | $t[$x[$i] * 16 + $y[$i]]];
    def digit($i): if $i >= 0 and $i < 16 then .[$i] else 0 end;
    # Shifts by $q digits and $r bits.
    def shl($q; $r): . as $x | [range(16) as $i | ($x | digit($i - $q)) * pow(2; $r) % 16 + (($x | digit($i - $q - 1)) / pow(2; 4 - $r) | floor)];
    def shr($q; $r): . as $x | [range(16) as $i | (($x | digit($i + $q)) / pow(2; $r) | floor) + ($x | digit($i + $q + 1)) * pow(2; 4 - $r) % 16];
  reduce range($work) as $_ (
    # 88172645463325252 is 0x0139408dcbbf7a44.
    {x: ([4, 4, 10, 7, 15, 11, 11, 12, 13, 8, 0, 4, 9, 3, 1, 0] | xor([range(16) as $i | $tick / pow(16; $i) | floor % 16])), sum: 0};
    .x |= (xor(shl(3; 1)) | xor(shr(1; 3)) | xor(shl(4; 1)))  # x ^= x << 13, x ^= x >> 7, x ^= x << 17
    | .x as $x | .sum += (reduce range(15; 2; -1) as $i (0; . * 16 + $x[$i]) * 2 + ($x[2] / 8 | floor) | sqrt))  # sqrt(x >> 11)
  | .x as $x | ((.sum | floor) + reduce range(15; 9; -1) as $i (0; . * 16 + $x[$i])) % 1000000;  # floor(sum) + (x >> 40)

def turn:
  .game as $game | .state as $state | $state.self as $me
  | ((.memory // {tracker: cy::tracker}).tracker | cy::track($state; $game | cy::tick_rate)) as $tracker
  | ($game | cy::is_3d and (cy::gravity | .z) == 0) as $free  # flying free, robots may be above or below
  | ($state | cy::nearest) as $target
  | ($tracker | cy::cue) as $cue  # the freshest passive contact, for a few seconds
  | cy::action | cy::forces({brake: 0.2})
  | if $target then
      cy::drive({throttle: (if $target.range > 350 then 0.7 elif $target.range < 200 then -0.4 else 0.2 end),
                 turn: cy::clamp(cy::angle_diff($target.bearing; $me.heading) / 45),
                 pitchTurn: cy::clamp(($target.elevation - $me.pitch) / 45)})
      | cy::scan($target.bearing; 20; $target.elevation)  # narrow: holds it, reaching about 735
      # The heaviest shot that still covers the range in about a second,
      # aimed where the target will be, lobbed under gravity.
      | ($game | cy::power_for_flight_time($target.range)) as $power
      | ($game | cy::lead($state; $target | cy::position($me.position); $tracker | cy::tracked_velocity($target.id); $power)
         // {heading: $target.bearing, pitch: $target.elevation}) as $aim
      | cy::aim($aim.heading; $aim.pitch)
      | if (cy::angle_diff($aim.heading; $me.aim.heading) | fabs) < 3 and (($aim.pitch - $me.aim.pitch) | fabs) < 3
           and ($game | cy::can_fire($state; $power; 10)) then cy::fire($power) else . end
      | cy::status("TRACK"; $target.range)
    elif $cue then
      # Head for where it was heard, sweeping an arc that still reaches it
      # across the bearing error (and, flying free, the elevation error).
      cy::bearing_to($me.position; $cue.position) as $bearing
      | cy::distance($me.position; $cue.position) as $distance
      | (if $free then cy::elevation_to($me.position; $cue.position) else 0 end) as $elevation
      | ([$cue.bearing_error, 30] | min) as $error
      | (($state.tick % 5 - 2) / 2 * $error) as $across
      | (if $free then (($state.tick / 5 | floor) % 3 - 1) * $error / 2 else 0 end) as $up
      | cy::drive({throttle: 0.7, turn: cy::clamp(cy::angle_diff($bearing; $me.heading) / 45),
                   pitchTurn: cy::clamp(($elevation - $me.pitch) / 45)})
      | cy::scan($bearing + $across; $game | cy::arc_for_reach($distance + 100); $elevation + $up)
      | cy::status("CUE"; $distance)
    else
      cy::drive({throttle: 0.5, turn: search_turn, pitchTurn: cy::clamp(-$me.pitch / 45)})
      # Sweeps the circle in half a second, flying free at three heights in turn.
      | cy::scan($state.sensors.scan.heading + 36; 40; if $free then (($state.tick / 10 | floor) % 3 - 1) * 35 else 0 end)
      | cy::status("SEARCH")
    end
  # Low battery: drive gently so the shots can still be paid for.
  | if $me.battery < $game.battery.capacity / 4 then cy::drive({throttle: (cy::get("drive"; "throttle") * 0.5)}) else . end
  # The storm hurts inside: drive straight away from its centre.
  | $state.storm as $storm
  | if $storm and ($storm | cy::contains($me.position; 60)) then
      cy::drive({throttle: 0.8, turn: cy::clamp(cy::angle_diff(cy::bearing_to($storm.center; $me.position); $me.heading) / 30)})
    else . end
  # Sudden death: head back inside the safe zone.
  | $state.zone as $zone
  | if $zone and ($zone | cy::margin($me.position)) < 60 then
      (if $zone.shape == "sphere" then cy::elevation_to($me.position; $zone.center) else 0 end) as $climb
      | cy::drive({throttle: 0.8, turn: cy::clamp(cy::angle_diff(cy::bearing_to($me.position; $zone.center); $me.heading) / 30),
                   pitchTurn: cy::clamp(($climb - $me.pitch) / 30)})
    else . end
  # Only the arena rate calibration sends calibration.work; it times this
  # language on the same work as every other. Safe to delete.
  | $state.calibration.work as $work
  | if ($work | type) == "number" and $work > 0 and $work == ($work | floor) then cy::status("WORK"; calibration_work($work; $state.tick)) else . end
  | {act: ., memory: {tracker: $tracker}};

cy::run(turn)'
