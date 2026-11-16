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
#
# Bearing is earned: a scan contact bearingError (degrees, a fixed share of
# the arc) says how far off its bearing may be, so a wide beam finds a robot
# and a narrow one aims at it. The robot narrows the beam on its target and
# shoots along a line fitted through its recent sightings. The jq SDK keeps
# no sighting history, so the robot keeps its own in memory. Contacts within
# 120 units are exact.
# Clunkyard robot protocol 2: one JSON object per line on stdin (hello, then
# a state each tick) and stdout (ready, then one act per state). Write
# debug output to stderr.
# hello: self {id name} match {mode ruleset control tickRate maxTicks
#   robots} arena {width height depth gravity terrain} robot {radius
#   maxHealth maxSpeed maxTurnRate maxPitchTurnRate maxPitch mass
#   momentOfInertia maxThrust maxTorque maxAngularSpeed brakeDeceleration
#   brakeAngularDeceleration mounts} battery {capacity regen
#   drivePerSecond strafeFactor compute} gun {turnRate power damage
#   escalation shotSpeed reloadTicks cost spread shotRadius shotRange
#   muzzleDistance burst} scanner {turnRate area minRange maxRange minArc
#   maxArc startArc proximityRange energyPerSecond warmupTicks bearing}
#   suddenDeath {startTick dps radiusStart radiusEnd shape center endTick}
#   sensing {faintFactor beam muzzle motionSpeed motionRange pingTicks
#   range bearingErrorPer100 rangeError rangeBin beamReach flashRange
#   fadeJitter noiseHold} objectives {score pickups hill} storm {radius
#   startTick speed growth dps} collisions {damagePerSpeed minSpeed
#   restitution cooldownTicks damageBy knock knockSeconds}
#   derived {hoverLift sustainableDrive}
# state: tick self {health battery position velocity heading pitch roll
#   angularVelocity attitude aim cooldownTicks spread grounded altitude
#   groundHeight applied} sensors {scan contacts} zone objectives {score
#   pickups armour hill} storm derived {self gun zone storm objectives}
# act, every part optional: tick drive {throttle turn pitchTurn}
#   forces {thrust strafe lift yawTorque pitchTorque rollTorque brake}
#   aim {heading pitch} scan {heading pitch arc on} fire {power}
#   status {text value} log
# ready: sdk avatar
# Angles are degrees from the +x axis; pitch is positive up. bot.json
# carries "protocol": 2.
# Manual: https://clunkyard.com/manual#protocol

# The jq program's $names are jq variables, not shell expansions.
# shellcheck disable=SC2016
exec jq -nc --unbuffered -L "$(dirname "$0")" '
import "clunkyard" as cy;

def search_turn: 0.33;  # enough to turn against the brake in the force rulesets
def search_arc: 40;     # degrees: sweeps the circle in half a second
def hold_width: 60;     # units across at the target: it stays in the beam from tick to tick
def min_arc: 6;         # degrees: the narrowest the beam holds
def window: 12;         # ticks of sightings the aim averages
def sightings: 8;       # narrow sightings in the window before a shot
def look_every: 40;     # ticks between looks around the target, for a nearer robot
def reacquire: 10;      # ticks to look again where a lost target was

# A straight line through the narrowest of the recent sightings of one robot:
# where it is now, how fast it moves, and how many sightings the line rests
# on. Each sighting is off by its bearingError; the line averages that out.
def fit($id; $now; $tick_rate):
  [.[] | select(.id == $id and .tick > $now - window)] as $recent
  | if ($recent | length) < 2 then null
    else
      ($recent | map(.bearingError) | min) as $best
      | [$recent[] | select(.bearingError <= 1.5 * $best)] as $seen
      | ($seen | length) as $n
      | if $n < 2 then null
        else
          (($seen | map(.tick) | add) / $n) as $t0
          | ($seen | map(pow(.tick - $t0; 2)) | add) as $spread
          | def axis($k):
              (($seen | map(.position[$k]) | add) / $n) as $mean
              | ((($seen | map((.tick - $t0) * (.position[$k] - $mean)) | add) / $spread) as $slope
                 | [$mean + $slope * ($now - $t0), $slope * $tick_rate]);
            axis("x") as [$x, $vx] | axis("y") as [$y, $vy] | axis("z") as [$z, $vz]
          | {position: {x: $x, y: $y, z: $z}, velocity: {x: $vx, y: $vy, z: $vz}, count: $n}
        end
    end;

def turn:
  .game as $game | .state as $state | $state.self as $me
  | ((.memory // {tracker: cy::tracker}).tracker | cy::track($state; $game | cy::tick_rate)) as $tracker
  | ($game | cy::is_3d and (cy::gravity | .z) == 0) as $free  # flying free, robots may be above or below
  | ($state | cy::nearest) as $target
  | ($tracker | cy::cue) as $cue  # the freshest passive contact, for a few seconds
  | (.memory.dodge // {end: 0, count: 0}) as $dodge
  # The target sightings of the last few ticks, this tick included.
  | ([(.memory.sightings // [])[] | select(.tick > $state.tick - window)]
     + (if $target then [{id: $target.id, tick: $state.tick, bearingError: ($target.bearingError // 0), position: ($target | cy::position($me.position))}] else [] end)) as $seen
  | (if $target then $target.id else .memory.target end) as $remembered
  | ($tracker | cy::last_seen($remembered)) as $last_seen
  | cy::action | cy::forces({brake: 0.2})
  | if $target then
      cy::drive({throttle: (if $target.range > 350 then 0.7 elif $target.range < 200 then -0.4 else 0.2 end),
                 turn: cy::clamp(cy::angle_diff($target.bearing; $me.heading) / 45),
                 pitchTurn: cy::clamp(($target.elevation - $me.pitch) / 45)})
      # Halve the beam each tick until it is hold_width across at the target:
      # narrow enough for a precise bearing, wide enough to keep it in.
      | ([min_arc, cy::degrees(2 * atan2(hold_width / 2; $target.range))] | max) as $hold
      # In a crowd a narrow beam sees one robot. Every look_every ticks,
      # widen it for a tick: a nearer robot it finds becomes the target.
      | (if $state.tick % look_every == 0 then search_arc
         else ([$hold, ([search_arc, $state.sensors.scan.arc / 2] | min)] | max) end) as $arc
      | cy::scan($target.bearing; $arc; $target.elevation)
      # The heaviest shot that still covers the range in about a second,
      # aimed where the target will be, lobbed under gravity.
      | ($game | cy::power_for_flight_time($target.range)) as $power
      # An exact contact: cy::lead is the intercept if the target holds
      # course. A noisy one: lead the fitted line, and fire only once it
      # rests on enough sightings.
      | (if ($target.bearingError // 0) > 0 then
           ($seen | fit($target.id; $state.tick; $game | cy::tick_rate)) as $line
           | {aim: (if $line then $game | cy::lead($state; $line.position; $line.velocity; $power) else null end),
              ready: ($line != null and $line.count >= sightings)}
         else
           {aim: ($game | cy::lead($state; $target | cy::position($me.position); $tracker | cy::tracked_velocity($target.id); $power)),
            ready: true}
         end) as $shot
      | ($shot.aim // {heading: $target.bearing, pitch: $target.elevation}) as $aim
      | cy::aim($aim.heading; $aim.pitch)
      | if $shot.ready and (cy::angle_diff($aim.heading; $me.aim.heading) | fabs) < 3 and (($aim.pitch - $me.aim.pitch) | fabs) < 3
           and ($game | cy::can_fire($state; $power; 10)) then cy::fire($power) else . end
      | cy::status(if $shot.ready then "TRACK" else "NARROW" end; $target.range)
    elif $last_seen != null and $state.tick - $last_seen <= reacquire then
      # Lost it: look again where it was last seen, widening the beam, before searching.
      ($tracker | cy::tracked_position($remembered)) as $where
      | cy::bearing_to($me.position; $where) as $bearing
      | (if $free then cy::elevation_to($me.position; $where) else 0 end) as $elevation
      | cy::drive({throttle: 0.5, turn: cy::clamp(cy::angle_diff($bearing; $me.heading) / 45),
                   pitchTurn: cy::clamp(($elevation - $me.pitch) / 45)})
      | cy::scan($bearing; ([search_arc, $state.sensors.scan.arc * 2] | min); $elevation)
      | cy::status("REACQUIRE")
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
      | cy::scan($state.sensors.scan.heading + 36; search_arc; if $free then (($state.tick / 10 | floor) % 3 - 1) * 35 else 0 end)
      | cy::status("SEARCH")
    end
  # A simple dodge: vary it, or predict the shooter.
  | (if $state.tick >= $dodge.end and any($state | cy::cues[]; .source == "muzzle")
     then {end: ($state.tick + 10), count: ($dodge.count + 1)} else $dodge end) as $dodge
  # Off the line of fire: left, then right next time (turn is positive left, strafe right).
  | ($dodge.count % 2 == 1) as $left
  | if $state.tick >= $dodge.end then .
    elif $game | cy::has_forces then cy::drive({throttle: 0.5}) | cy::forces({strafe: (if $left then -1 else 1 end)})
    else cy::drive({throttle: 1, turn: (if $left then 1 else -1 end)}) end
  | if $state.tick < $dodge.end then cy::status(if $left then "dodge:left" else "dodge:right" end; $dodge.count) else . end
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
  | {act: ., memory: {tracker: $tracker, dodge: $dodge, sightings: $seen, target: $remembered}};

cy::run(turn)'
