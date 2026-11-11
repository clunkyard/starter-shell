# Clunkyard robot SDK for the shell: protocol 2 as a jq module.
#
# A shell starts a process for nearly everything, and a process per turn
# costs more CPU time than a match allows, so the robot is one long-running
# jq and this module is its SDK. The robot's script imports it and hands its
# turn to run:
#
#   #!/bin/sh
#   exec jq -nc --unbuffered -L "${0%/*}" '
#     import "clunkyard" as cy;
#     def turn:
#       .state as $state
#       | ($state | cy::nearest) as $target
#       | {act: (cy::action | if $target then cy::aim($target.bearing; $target.elevation) | cy::fire else . end)};
#     cy::run(turn)'
#
# jq has no classes, so the SDK is a set of definitions on plain JSON:
#   the game     is the hello itself:   $game | cy::reach(20)
#   the state    is the state message:  $state | cy::nearest, $state | cy::cues
#   an action    is an object built by piping it through the controls:
#                cy::action | cy::drive({throttle: 0.5}) | cy::aim(90; 0) | cy::fire
#   memory       is whatever JSON the robot returns, handed back next turn
# The robot's turn reads {game, state, memory} and returns {act, memory};
# leave out memory to keep the last one. Every message is the raw protocol
# JSON (protocol/v2/schema.json), so a field this SDK does not name is one
# path away, and cy::extra adds raw fields to an act.
#
# Angles are degrees. Headings run from the +x axis toward +y, 0 to 360;
# pitch and elevation are positive upward. Vectors are {x, y, z} objects.
# jq's % works on integers only, so angles wrap with floor.

def sdk_name: "clunkyard-shell";
def sdk_version: "2.1.0";
def protocol: 2;

# Writes a debug line to standard error and passes its input through.
def log($message): (($message | tostring) + "\n" | stderr | empty), .;

# Gives up the match from a turn: the SDK answers with a forfeit message.
def forfeit($code; $message): error({forfeit: {code: $code, message: $message}});
def forfeit($code): forfeit($code; "");

# --- Geometry ---------------------------------------------------------------

def pi: 1 | atan * 4;
def radians($degrees): $degrees * pi / 180;
def degrees($radians): $radians * 180 / pi;
def clamp($value; $low; $high): if $value < $low then $low elif $value > $high then $high else $value end;
def clamp($value): clamp($value; -1; 1);
# A heading in 0..360.
def normalize($degrees): $degrees - 360 * ($degrees / 360 | floor);
# The signed turn from current to target, in -180..180.
def angle_diff($target; $current): normalize($target - $current + 180) - 180;

def vec($x; $y; $z): {x: $x, y: $y, z: $z};
def vec($v): {x: ($v.x // 0), y: ($v.y // 0), z: ($v.z // 0)};
def vadd($a; $b): {x: ($a.x + $b.x), y: ($a.y + $b.y), z: (($a.z // 0) + ($b.z // 0))};
def vsub($a; $b): {x: ($a.x - $b.x), y: ($a.y - $b.y), z: (($a.z // 0) - ($b.z // 0))};
def vscale($a; $k): {x: ($a.x * $k), y: ($a.y * $k), z: (($a.z // 0) * $k)};
def dot($a; $b): $a.x * $b.x + $a.y * $b.y + ($a.z // 0) * ($b.z // 0);
def vlength($v): dot($v; $v) | sqrt;
# Length in the x-y plane.
def planar($v): $v.x * $v.x + $v.y * $v.y | sqrt;
def distance($a; $b): vlength(vsub($a; $b));
# The unit vector for a heading and pitch.
def direction($heading; $pitch):
  radians($heading) as $h | radians($pitch) as $p
  | {x: (($p | cos) * ($h | cos)), y: (($p | cos) * ($h | sin)), z: ($p | sin)};
def direction($heading): direction($heading; 0);
def bearing_to($origin; $target): normalize(degrees(atan2($target.y - $origin.y; $target.x - $origin.x)));
def elevation_to($origin; $target): vsub($target; $origin) as $d | degrees(atan2($d.z; planar($d)));

# --- Aiming -----------------------------------------------------------------

# The earliest flight time t at which a shot fired now meets the target:
# |D(t) - g t^2 / 2| = speed t + muzzle, where D is the target's position
# relative to the shooter at t. Takes {shooter, target, shot_speed} and
# optionally target_velocity, shooter_velocity, gravity, muzzle and
# max_time (5 s); returns {heading, pitch, time}, or null out of reach.
def solve_lead($o):
  vec($o.shooter) as $s | vec($o.target) as $target | vec($o.target_velocity) as $tv
  | vec($o.shooter_velocity) as $sv | vec($o.gravity) as $g
  | $o.shot_speed as $speed | ($o.muzzle // 0) as $muzzle | ($o.max_time // 5) as $max
  | def gap($t): vsub(vadd($target; vscale($tv; $t)); vadd(vadd($s; vscale($sv; $t)); vscale($g; 0.5 * $t * $t)));
    def miss($t): vlength(gap($t)) - ($speed * $t + $muzzle);
    (if miss(0) <= 0 then 0
     else
       (first(range(1; 65) | ($max * . / 64) as $t | select(miss($t) <= 0) | [$max * (. - 1) / 64, $t]) // null)
       | if . == null then null
         else reduce range(40) as $_ (.; ((.[0] + .[1]) / 2) as $mid | if miss($mid) > 0 then [$mid, .[1]] else [.[0], $mid] end) | .[1]
         end
     end) as $time
    | if $time == null then null
      else gap($time) as $d
      | if vlength($d) == 0 then {heading: 0, pitch: 0, time: $time}
        else {heading: normalize(degrees(atan2($d.y; $d.x))), pitch: degrees(atan2($d.z; planar($d))), time: $time}
        end
      end;

# --- hello: the game, with the rules' arithmetic -----------------------------
# Each of these reads the hello as its input: $game | cy::shot_speed(1).

def tick_rate: .match.tickRate // 20;
def max_ticks: .match.maxTicks // 1800;
def is_3d: .match.mode == "3d";
# True in the force rulesets, where controls are forces and momentum carries.
def has_forces: .match.control == "forces";
def gravity: vec(.arena.gravity);

def seconds($ticks): $ticks / tick_rate;
def remaining_ticks($tick): [0, max_ticks - $tick] | max;

def linear($line; $power): ($line.base // 0) + ($line.perPower // 0) * $power;
# A legal power; null gives gun.power.default.
def clamp_power($power): .gun.power as $p | clamp($power // $p.default; $p.min; $p.max);
def shot_speed($power): linear(.gun.shotSpeed; clamp_power($power));
def shot_cost($power): linear(.gun.cost; clamp_power($power));
def reload_ticks($power): linear(.gun.reloadTicks; clamp_power($power)) + 0.5 | floor;
# Damage of a hit at this power landing at this tick, escalation included.
def shot_damage($power; $tick):
  .gun.escalation as $e | ([0, $tick - ($e.startTick // 0)] | max) as $late
  | linear(.gun.damage; clamp_power($power)) * (1 + ($e.perSecond // 0) * $late / tick_rate);
def shot_damage($power): shot_damage($power; 0);
# The heaviest shot that covers range in about `seconds`.
def power_for_flight_time($range; $seconds):
  .gun.shotSpeed as $s
  | if ($s.perPower // 0) == 0 then clamp_power(null) else clamp_power(($range / $seconds - $s.base) / $s.perPower) end;
def power_for_flight_time($range): power_for_flight_time($range; 1);

def clamp_arc($arc): clamp($arc; .scanner.minArc; .scanner.maxArc);
# How far a scan of this arc reaches.
def reach($arc): .scanner as $s | clamp($s.area / clamp_arc($arc) | sqrt; $s.minRange; $s.maxRange);
# The widest scan arc that still reaches range.
def arc_for_reach($range): clamp_arc(.scanner.area / ([1e-9, $range * $range] | max));

# Battery per second to hold a drive fraction (0 to 1).
def drive_cost($drive): .battery.drivePerSecond * $drive * $drive;
# The drive fraction the battery's regeneration pays for forever.
def sustainable_drive:
  .battery as $b | if ($b.drivePerSecond // 0) <= 0 then 1 else [1, ($b.regen / $b.drivePerSecond | sqrt)] | min end;
# The forces.lift that cancels gravity for a level robot.
def hover_lift:
  .robot.maxThrust as $thrust
  | if ($thrust // 0) == 0 then 0 else [1, .robot.mass * -(gravity | .z) / $thrust] | min end;
# True if a shot at this power leaves this tick, keeping `reserve` charge back.
def can_fire($state; $power; $reserve):
  ($state.self.cooldownTicks // 0) == 0 and ($state.self.battery // 0) >= shot_cost($power) + $reserve;
def can_fire($state; $power): can_fire($state; $power; 0);
def can_fire($state): can_fire($state; null; 0);

# Where to aim to hit a target (a position) moving at velocity, allowing for
# gravity and, in the force rulesets, your own velocity, which shots
# inherit. null when the shot cannot reach it.
def lead($state; $target; $velocity; $power):
  shot_speed($power) as $speed
  | solve_lead({shooter: $state.self.position, target: $target, target_velocity: $velocity, shot_speed: $speed,
      shooter_velocity: (if has_forces then $state.self.velocity else null end), gravity: gravity,
      muzzle: .gun.muzzleDistance, max_time: (.gun.shotRange / ([1, $speed] | max))});
def lead($state; $target; $velocity): lead($state; $target; $velocity; null);

# --- state ------------------------------------------------------------------
# Each of these reads the state message as its input: $state | cy::nearest.
# A block the arena leaves out (zone before sudden death, storm, objectives
# with the mechanics off) reads as null, so `if $state.storm then` works.

def me: .self;
# True for a contact that names its robot, which you can aim at. A passive
# contact (faint, beam, muzzle, motion, ping) has a null id and a noisy
# bearing: it says which way to look, not where to shoot.
def precise: (.id | type) == "string" and (.id | length) > 0;
def contacts: .sensors.contacts // [];
# The nearest precise contact, or null. Passive contacts are never targets.
def nearest: [contacts[] | select(precise)] | min_by(.range);
# The precise contact for this robot id, or null.
def contact($id): first(contacts[] | select(precise and .id == $id)) // null;
# The passive contacts, most accurate (smallest bearingError) first.
def cues: [contacts[] | select(precise | not)] | sort_by(.bearingError // 0);
# Where a contact (its input) is, seen from origin (your position).
def position($origin): vadd(vec($origin); vscale(direction(.bearing; .elevation // 0); .range));
# state.objectives with its parts defaulted: no score, no pickups, no armour, no hill.
def objectives: {score: null, pickups: [], armour: 0, hill: null} + (.objectives // {});

# The zone, the storm and the hill share these; each reads the area as its
# input. The storm ignores height in every mode, a column ignores it, and a
# sphere (the hill in zero-g, the zone in 3D) counts it.
def distance_from($point):
  vsub(vec($point); vec(.center)) as $d
  | if .shape == "column" or has("dps") then planar($d) else vlength($d) end;
# How far inside the area point is; negative outside.
def margin($point): .radius - distance_from($point);
# True if point is inside, or within margin of the edge.
def contains($point; $margin): distance_from($point) <= .radius + $margin;
def contains($point): contains($point; 0);

# Distance as pickups count it (the hello's input): planar when the field is
# a circle or a column, in 3D when it is a sphere.
def field_distance($a; $b):
  vsub(vec($a); vec($b)) as $d | if .suddenDeath.shape == "sphere" then vlength($d) else planar($d) end;
# True if point (your position unless you name one) is in the storm.
def in_storm($state; $point): $state.storm != null and ($state.storm | contains($point // $state.self.position));
def in_storm($state): in_storm($state; null);
# The pickup nearest you, of this kind (energy, health, armour) unless null.
def nearest_pickup($state; $kind):
  . as $game
  | [$state | objectives | .pickups[] | select($kind == null or .kind == $kind)]
  | min_by(.position as $p | $game | field_distance($p; $state.self.position));
def nearest_pickup($state): nearest_pickup($state; null);
# True if you are on the hill; holding it also needs nobody else there.
def on_hill($state): ($state | objectives | .hill) as $hill | $hill != null and ($hill | contains($state.self.position));

# --- act --------------------------------------------------------------------
# An action is an object piped through the controls: only what you set is
# sent. Left out, drive and forces are 0, aim and scan keep where they
# point, and no shot is fired. A value must be a finite number (null leaves
# it unset); the SDK clamps it to its legal range when it writes the act.

def action: {};

def _finite($where):
  if . == null then .
  elif type != "number" or isinfinite or isnan then error("\($where) must be a finite number, not \(tojson)")
  else . end;
def _set($group; $values):
  reduce ($values | to_entries[] | select(.value != null)) as $e (.;
    .[$group][$e.key] = ($e.value | _finite("\($group).\($e.key)")));

# Direct controls, -1 to 1: {throttle, turn, pitchTurn}. In the force
# rulesets they stand in for thrust, yaw torque and pitch torque left out.
def drive($controls): _set("drive"; $controls);
# Force controls, -1 to 1 (brake 0 to 1): {thrust, strafe, lift, yawTorque,
# pitchTorque, rollTorque, brake}.
def forces($controls): _set("forces"; $controls);
# Where to turn the turret.
def aim($heading; $pitch): _set("aim"; {heading: $heading, pitch: $pitch});
def aim($heading): aim($heading; null);
# Where to point the scanner for the next state, and how wide.
def scan($heading; $arc; $pitch): _set("scan"; {heading: $heading, pitch: $pitch, arc: $arc});
def scan($heading; $arc): scan($heading; $arc; null);
# Fire this tick, at gun.power.default unless you name a power.
def fire($power): if $power == null then .fire = {} else .fire = {power: ($power | _finite("fire.power"))} end;
def fire: fire(null);
def hold_fire: del(.fire);
def firing: has("fire");
# A status line for spectators; text up to 32 characters.
def status($text; $value):
  .status = ({} + (if $text == null then {} else {text: ($text | tostring)} end)
                + (if $value == null then {} else {value: ($value | _finite("status.value"))} end));
def status($text): status($text; null);
# Raw top-level fields for the act: the escape hatch for protocol fields
# this SDK does not model.
def extra($fields): ._extra = ((._extra // {}) + $fields);
# What was set for one control, or null.
def get($group; $name): .[$group][$name]?;

def _clamp_control($group; $name; $value; $game):
  if $group == "drive" or $group == "forces" then clamp($value; if $name == "brake" then 0 else -1 end; 1)
  elif $name == "heading" then normalize($value)
  elif $name == "pitch" then (($game.robot.maxPitch? // 80)) as $limit | clamp($value; -$limit; $limit)
  elif $name == "arc" and $game != null then $game | clamp_arc($value)
  else $value end;

# The act message for this tick, clamped to the legal ranges; game may be null.
def to_message($tick; $game):
  . as $act
  | reduce ("drive", "forces", "aim", "scan") as $group ({v: protocol, type: "act", tick: $tick};
      if ($act[$group] // {}) == {} then .
      else .[$group] = ($act[$group] | with_entries(.value = _clamp_control($group; .key; .value; $game))) end)
  | if $act | has("fire") then
      .fire = (if $game != null and ($act.fire | has("power")) then {power: ($game | clamp_power($act.fire.power))} else $act.fire end)
    else . end
  | if $act | has("status") then .status = $act.status else . end
  | . + ($act._extra // {});

# --- tracking ---------------------------------------------------------------
# A tracker is plain JSON the robot keeps in its memory. It estimates each
# precise contact's position and velocity from the ticks it was seen, and
# remembers the freshest passive contact as a cue for cue_ticks, since a
# passive contact lasts one tick.

def tracker($options): {smoothing: 0.5, forget_ticks: 40, cue_ticks: 60, tick: 0, tracks: {}, cue: null} + $options;
def tracker: tracker({});

# The tracker (its input) after this state.
def track($state; $tick_rate):
  ($state.self.position) as $me
  | ($state | cues) as $cues
  | .tick = $state.tick
  | if ($cues | length) > 0 then
      .cue = ($cues[0] | {position: position($me), tick: $state.tick, source, bearing_error: (.bearingError // 0)})
    else . end
  | reduce ($state | contacts[] | select(precise)) as $c (.;
      ($c | position($me)) as $position | .tracks[$c.id] as $t
      | if $t != null and ($state.tick - $t.tick) > 0 and ($state.tick - $t.tick) <= .forget_ticks then
          (($state.tick - $t.tick) / $tick_rate) as $dt
          | vscale(vsub($position; $t.position); 1 / $dt) as $measured
          | (if $t.seen > 1 then vadd(vscale($t.velocity; .smoothing); vscale($measured; 1 - .smoothing)) else $measured end) as $velocity
          | .tracks[$c.id] = {tick: $state.tick, position: $position, velocity: $velocity, seen: ($t.seen + 1)}
        else .tracks[$c.id] = {tick: $state.tick, position: $position, velocity: vec(0; 0; 0), seen: 1} end);
def track($state): track($state; 20);

def tracked_position($id): if $id == null then null else .tracks[$id].position end;
def tracked_velocity($id): (if $id == null then null else .tracks[$id].velocity end) // vec(0; 0; 0);
def last_seen($id): if $id == null then null else .tracks[$id].tick end;
# The freshest passive contact heard within cue_ticks: {position, tick,
# source, bearing_error}, or null.
def cue: if .cue != null and .tick - .cue.tick <= .cue_ticks then .cue else null end;

# --- robot and loop ---------------------------------------------------------

# The protocol without input and output. Its input is {session, message},
# where session is {game, memory} (null at first); it returns {session,
# reply}, with a null reply for a message that needs none. turn reads
# {game, state, memory} and returns {act, memory}. An error in turn is
# logged and answered with an empty act, so one bad tick does not end the
# match; cy::forfeit answers with a forfeit message.
def handle(turn):
  (.session // {game: null, memory: null}) as $session | .message as $message
  | if $message.type == "hello" then
      # A protocol 1 hello carries "v":2 under rules v2, so its shape decides.
      if $message.v != protocol or ($message.match | type) != "object" then
        error("this SDK speaks protocol \(protocol); add \"protocol\": 2 to bot.json")
      else {session: {game: $message, memory: null},
            reply: {v: protocol, type: "ready", sdk: {name: sdk_name, version: sdk_version}}} end
    elif $message.type == "state" then
      ($message.tick // 0) as $tick
      | try (
          ({game: $session.game, state: $message, memory: $session.memory} | turn) as $out
          | if ($out | type) != "object" or ($out.act | type) != "object" then
              error("turn returned \($out | tojson | .[:80]), not {act, memory}") else . end
          | {session: ($session | if $out | has("memory") then .memory = $out.memory else . end),
             reply: ($out.act | to_message($tick; $session.game))})
        catch (
          if (type == "object") and has("forfeit") then
            {session: $session, reply: {v: protocol, type: "forfeit", tick: $tick, code: .forfeit.code, message: .forfeit.message}}
          else
            log("tick \($tick): turn failed: \(if type == "string" then . else tojson end)")
            | {session: $session, reply: {v: protocol, type: "act", tick: $tick}}
          end)
    else {session: $session, reply: null} end;

# Plays one match: answers every message on standard input (run jq with -n).
def run(turn):
  foreach inputs as $message ({session: null};
    {session: .session, message: $message} | handle(turn);
    .reply // empty);

# Feeds recorded messages (its input: an array of a hello, then states) to a
# turn and returns its replies, for deterministic tests without a match.
def play(turn):
  [foreach .[] as $message ({session: null};
    {session: .session, message: $message} | handle(turn);
    .reply // empty)];
