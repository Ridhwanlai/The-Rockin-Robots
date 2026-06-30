
// out of 20 teams in the course, we placed 2nd. we placed 3rd with our speed... 

# Bob (Best Of the Best) — Our COGS 300 Autonomous Robot
 
> *"we built the chinese ev version of our robot"* — me, the morning of the tournament
 
## Table of Contents
 
- [Introduction and Background](#introduction-and-background)
  - [Who We Are](#who-we-are)
  - [Why We Built This](#why-we-built-this)
  - [The Course, and What "Autonomous" Actually Meant for Us](#the-course-and-what-autonomous-actually-meant-for-us)
- [Meet Bob](#meet-bob)
  - [What Bob Is](#what-bob-is)
  - [The Build — Chassis, Brains, and Senses](#the-build--chassis-brains-and-senses)
  - [The Concepts We Leaned On](#the-concepts-we-leaned-on)
- [The Three Challenges](#the-three-challenges)
  - [Part 1 — Line Following](#part-1--line-following)
  - [Part 2 — Wall Following](#part-2--wall-following)
  - [Part 3 — Object Detection](#part-3--object-detection)
  - [Tying It Together — The State Machine](#tying-it-together--the-state-machine)
- [The Strategy (or, How We Decided to Lose on Purpose)](#the-strategy-or-how-we-decided-to-lose-on-purpose)
- [Challenges — Explained Per Section](#challenges--explained-per-section)
- [Reflections](#reflections)
- [Repository Files](#repository-files)
- [References & Citations](#references--citations)
---
 
## Introduction and Background
 
### Who We Are
 
Hey! We're **Adrian, Ella, Jayson, Ridhwan, and Yui** — five UBC students who spent 12 weeks turning a pile of acrylic, jumper wires, and yellow TT motors into a robot we named **Bob**, short for **Best Of the Best**. (Was Bob actually the best of the best? Stick around and find out. The name aged… interestingly.)
 
This is the story of how Bob came to be: every late-night lab session, every dead battery, every line he skipped, and every time he hit the cheese. If you've never touched an Arduino in your life, perfect — neither had most of us when we started. By the end you'll know exactly how a little two-wheeled robot can follow a line, hug a wall, and go hunting for a block of cardboard cheese, all on its own.
 
### Why We Built This
 
This was our final project for **COGS 300: Understanding and Designing Cognitive Systems** at UBC — a course where, over a single term, you go from "I've maybe seen a `for` loop once" to building a fully autonomous robot that has to survive a real obstacle course in front of a crowd. The whole premise of the class is that you don't really understand intelligence — sensing, deciding, acting — until you've tried to build it yourself and watched it fail in a hundred small, specific, deeply humbling ways.
 
And fail it did. And then it worked. That arc — from "why is it spinning in a circle" to "oh my god it's doing it" — is the entire point, and it's what this post is about.
 
### The Course, and What "Autonomous" Actually Meant for Us
 
COGS 300 frames robotics through *embodied cognition*: the idea that intelligence isn't just code sitting in a vacuum, it's an agent **coupled to its environment** through sensors and motors.<sup>[1](#footnotes)</sup> Bob doesn't "know" where he is on a map. He has no GPS, no camera, no grand internal model of the world. He has three little infrared eyes pointed at the floor, two ultrasonic sensors pinging the space around him, and a set of simple rules. Intelligence, here, is *emergent* — it falls out of the tight loop between what Bob senses right now and what he does about it a few milliseconds later.
 
That's the lens we kept coming back to: **good behaviour didn't come from a smarter brain, it came from a tighter, cleaner sense→decide→act loop.** Almost every breakthrough in this project was us shortening that loop or cleaning up the signal feeding into it.
 
The competition itself had **three parts**, run as one continuous course:
 
1. **Line-following** — follow a winding strip of tape from the start.
2. **Wall-following** — navigate a cardboard maze by hugging a wall.
3. **Object-detection** — find and hit a target object (lovingly referred to as "the cheese") at the end.
You were timed start-to-finish, with penalties if a human had to intervene. Three challenges, one robot, one run. Easy, right? (It was not easy.)
 
> **Figure 1:** The full tournament track, stitched together from overhead drone shots — the taped line-following section, the cardboard-walled maze for wall-following, and the open object-detection arena at the end.
>
> _(see `/drone-shots/full_track_stitch.jpg`)_
 
---
 
## Meet Bob
 
### What Bob Is
 
Bob is a **two-wheeled differential-drive robot** built on an Arduino Uno. "Differential drive" is a fancy way of saying he steers like a tank: two independently-driven wheels, and to turn you just spin one faster than the other. There's no steering wheel, no servo turning an axle — turning *is* a speed difference between left and right. This one fact shapes basically every line of driving code we wrote.
 
> **Figure 2:** Bob in the flesh — the two-tier acrylic chassis, a yellow TT-motor drive wheel, the breadboard up front, and the beautiful spaghetti of jumper wires that somehow all mattered.
>
> _(see `/general/IMG_5019.jpg`)_
 
Here's the rough mapping of "robot part → what it does," in the same spirit as thinking about an agent's body:
 
| Robot Part | What It Does (the cognitive role) |
| --- | --- |
| 3× IR sensors (floor-facing) | "Eyes" for the line — is there tape under me, left/middle/right? |
| Front ultrasonic (HC-SR04) | "Is there a wall right in front of me?" |
| Right ultrasonic (HC-SR04) | "How far am I from the wall on my right?" |
| Arduino Uno | The brain — runs the sense→decide→act loop |
| L298N motor driver | The "spinal cord" — turns brain signals into real motor power |
| 2× TT motors + wheels | The muscles — actually move Bob |
| Steel-ball caster | The third contact point, so Bob doesn't tip (upgraded from a janky "hammer caster" — more on that later) |
| Battery pack + rocker switch | Bob's heart and on/off button |
 
> **Figure 3:** The HC-SR04 ultrasonic sensor (those two little "eyes") next to the steel-ball caster wheel that replaced our original wobbly caster.
>
> _(see `/general/IMG_5372.jpg`)_
 
### The Build — Chassis, Brains, and Senses
 
The physical build went through real iterations. A few decisions that mattered more than we expected:
 
- **We added a second chassis layer.** Stacking a second acrylic deck (held up by brass standoffs) gave us room to mount the breadboard, the Arduino, the motor driver, and the battery pack without everything fighting for space. A tidy robot is a debuggable robot.
- **We obsessed over stabilizing components.** Early on, the battery pack and motor driver would shift around mid-run, which quietly wrecks your wiring and your sanity. We ended up using proper double-coated tissue tape (shoutout `3M 9448A`) to lock things down. Boring fix, huge payoff.
- **We swapped the caster.** Our first rear support was a hammer-style caster that added drag and made turns inconsistent. A **steel ball caster** rolls in every direction freely, and the difference in how cleanly Bob turned was night and day.
- **Batteries became a whole storyline.** We went through *a lot* of batteries. At one point we promised, on record, that "these batteries and the battery charger will never leave the lab from now on." (They left the lab.) Lithium prices are real, folks.
> **Figure 4:** A teammate sourcing components — the unglamorous, deeply necessary supply runs to Lee's Electronics that kept Bob alive.
>
> _(see `/general/IMG_5380.jpg`)_
 
### The Concepts We Leaned On
 
A few core ideas from the course did the heavy lifting throughout. I want to name them up front, because the rest of this post is really just these concepts showing up over and over in different costumes:
 
- **Sense → Decide → Act loops.** The fundamental rhythm of the whole robot. Read sensors, pick a behaviour, drive the motors, repeat — dozens of times per second.
- **Reactive / behaviour-based control.** Bob doesn't plan a route. Each behaviour ("follow the line," "hug the wall") is a direct reflex mapping current sensor readings to motor commands. Complex-looking navigation falls out of simple reflexes stacked together.<sup>[2](#footnotes)</sup>
- **Finite state machines (FSMs).** Bob is always in exactly one *mode* — line-following, wall-following, or halted — and well-defined conditions trigger transitions between them. This is what let us glue three totally different behaviours into one continuous run.
- **A deliberate right-turn bias.** We baked a structural preference for the right into Bob. When he lost the line, he'd arc gently *right* to find it again; in the maze he followed the *right* wall. Committing to one side turns an ambiguous "which way do I go??" into a single, predictable, tunable behaviour. (More on why this was secretly our best decision later.)
- **Sensor smoothing (moving averages).** Raw ultrasonic readings are noisy and occasionally insane. We ran them through a simple moving average so one garbage ping couldn't make Bob panic.
- **Feedback & corrective control.** Wall-following is basically a hand-built proportional-ish controller: measure the error (am I too close / too far from the wall?), and correct in the opposite direction. Classic closed-loop thinking.
- **Telemetry-driven debugging.** We had Bob constantly print what he was sensing and deciding over serial. You cannot fix what you cannot see, and this turned "the robot is being weird" into "ah, the right sensor reads 76cm at this exact spot."
Now for the fun part — drumroll please — let's actually walk through the three challenges. And hey, if any of the above didn't fully land, don't sweat it. It didn't land for us either, at first. :)
 
---
 
## The Three Challenges
 
### Part 1 — Line Following
 
**The goal:** Bob starts on a strip of tape that winds across the floor — gentle curves, sharp turns, the occasional cruel hairpin — and has to follow it without losing it.
 
**How it works.** Bob has **three downward-facing IR sensors**: left, middle, right. Each one answers a yes/no question — "is there a line under me?" — and from those three bits Bob decides how to steer:
 
- **Middle sees the line, sides don't?** Great, you're centered — drive straight.
- **Right sensor catches the line?** The line is drifting right, so turn right to recenter.
- **Left sensor catches the line?** Turn left.
- **Nobody sees the line?** You've lost it — enter **seek mode**.
That last case is where our **right-turn bias** lives, and it's the single most important design choice in the line-follower. When Bob loses the line entirely, he doesn't stop and he doesn't reverse — he drives in a **gentle rightward arc** until a sensor catches tape again, by running the left wheel slightly faster than the right:
 
```cpp
// When the line is lost: left wheel slightly faster than right → gentle rightward drift.
const int LF_SEEK_LEFT  = 105;   // left wheel speed while seeking line
const int LF_SEEK_RIGHT = 88;    // right wheel speed while seeking line
```
 
The *gap* between those two numbers is the whole personality of the seek behaviour. Widen it and Bob carves a sharp arc (good for tight turns, but he'll overshoot a straight line). Narrow it and he drifts lazily (smooth, but he'll sail right off a hairpin). We spent an genuinely embarrassing number of hours tuning these two integers.
 
> **Figure 5:** The line-following maze as seen mid-run — you can see the zigzag and hexagon shapes in white tape, plus Bob's live serial telemetry on the left (`[LF] L=1 M=1 R=1 Front=68.5cm Right=76.9cm`).
>
> _(see `/bob/Screenshot_2026-04-04_at_9.05.00_AM.png`)_
 
**Concepts in play:** reactive control (three bits in, a steering reflex out), the right-turn bias as a tie-breaker, and a tiny bit of feedback control in how aggressively we correct.
 
### Part 2 — Wall Following
 
**The goal:** A maze made of cardboard walls. No tape to follow now — Bob has to thread the corridors using only his sense of the walls around him.
 
**The strategy: follow the right wall.** There's a classic result that if you *always* keep one hand on the same wall of a maze, you'll eventually get through it. We made that Bob's whole philosophy. He uses his **right ultrasonic sensor** to maintain a target distance from the right wall, and his **front ultrasonic sensor** to notice when he's about to crash into something ahead.
 
The decision logic, in priority order, looks like this:
 
1. **Stuck?** If Bob hasn't moved in a while, run a recovery routine (more below).
2. **Wall close in front, wall on the right?** You're in a corner — turn left.
3. **Wall close in front, nothing on the right?** Turn right (the corridor opened up to your right).
4. **Front clear, right wall visible?** Do distance-keeping: too close → nudge left, too far → nudge right, just right → go straight.
5. **Front clear, no right wall at all?** Open space — just drive straight.
That step 4 is a little **feedback controller** by hand. The "just right" zone is a tolerance band around our target distance, so Bob doesn't jitter back and forth forever:
 
```cpp
const float WF_RIGHT_NEAR_CM      = 8.0f;   // target distance from the right wall
const float WF_RIGHT_TOLERANCE_CM = 1.0f;   // dead-band so Bob doesn't oscillate
```
 
**Smoothing the senses.** Ultrasonic sensors lie sometimes — a single bad echo can read 400cm when the wall is clearly 8cm away. If Bob reacted to every reading, he'd convulse. So every distance goes through a **simple moving average** before he trusts it:
 
```cpp
#define US_SMA_WINDOW 4   // average the last 4 readings to kill noise spikes
```
 
**The stuck-detection saga.** Bob's nemesis in the maze was *getting wedged* — nose against a wall, wheels spinning, going nowhere, with his sensor readings frozen. We built a **two-tier recovery system**:
 
- **Tier 1 (first time stuck):** a firm straight reverse, then a strong push forward to break free.
- **Tier 2 (still stuck):** an *angled* reverse — backing up while turning — to wiggle out of the corner instead of just re-ramming the same wall.
This was the difference between "Bob gets stuck once and the run is over" and "Bob frees himself and keeps going." We literally watched the serial log narrate it in real time:
 
```
[WALL] Front=69.8cm (FAR)  Right=10.4cm (TOO FAR)  [STUCK-N]  ->  [STUCK-N] Soft reverse...
[STUCK-N] Done.
```
 
**Concepts in play:** feedback/corrective control, the right-wall rule as a navigation strategy, sensor smoothing, and stateful recovery behaviours.
 
### Part 3 — Object Detection
 
**The goal:** At the end of the course, Bob has to find a target object and make contact with it. There were a few possible objects on the table — the **cheese** (short, close to the ground), the **TNT** (taller, wide flat profile), and the **clock tower** (tall, narrow profile). The cheese was our friend; the others were traps.
 
**The clever bit — repurposing avoidance into seeking.** Here's a thing I'm proud of us for. There's a *ton* of example code online for robots that **avoid** obstacles — they're everywhere, it's the "hello world" of ultrasonic robots. There is far less code for robots that deliberately **hunt** an object. Rather than start from a blank file, we took a well-tested obstacle-*avoidance* program and **inverted its logic**: instead of "object detected → steer away," we made it "object detected → drive straight at it." Same sensors, same structure, opposite goal. Why reinvent the wheel when you can flip it around?
 
For telling objects apart, the plan combined the IR sensor (does it trigger low to the ground?) with an **ultrasonic sweep** to read the object's *profile*:
 
```
Cheese (5")  → IR triggers (it's low to the ground)
TNT (10")    → IR triggers BUT ultrasonic sweep shows a wide, flat profile
Clock Tower  → IR triggers AND ultrasonic shows a narrow profile
```
 
A short flat thing is the cheese; a wide flat thing is the TNT; a narrow tall thing is the clock tower. Geometry as identity.
 
> **Figure 6:** Early object-detection testing in the lab — dialing in how Bob approaches and makes contact with the target.
>
> _(see `/bob/IMG_5732.mov`)_
 
**Concepts in play:** reusing and re-framing an existing behaviour (avoidance → seeking), and multi-sensor fusion (IR + ultrasonic profile) to classify what Bob is looking at.
 
### Tying It Together — The State Machine
 
Three behaviours are nice, but the tournament was **one continuous run**. The thing that made Bob *one robot* instead of three half-robots was a **finite state machine**. At any instant Bob is in exactly one mode, and specific sensor conditions flip him from one to the next:
 
```cpp
enum RobotMode {
  MODE_LINE_FOLLOW,   // Part 1: follow the tape
  MODE_WALL_FOLLOW,   // Part 2: hug the right wall
  MODE_HALT           // done / exit
};
RobotMode currentMode = MODE_LINE_FOLLOW;   // Bob always starts on the line
```
 
The transitions were the genuinely hard part — way harder than any single behaviour. Knowing *when* the line has truly ended and the maze has begun (versus Bob just briefly losing the line on a curve) took an enormous amount of tuning. We used a short "coast" delay and a check for a wall appearing on the right to confirm a real handoff from line-following to wall-following, so a momentary blip wouldn't trip a premature mode switch.
 
This was the heart of our homework the week the prof told us to *"build a model of the tournament you can test and debug with… make it modular and updatable."* Modular was the magic word: by keeping each behaviour's constants **completely separate** (line-following speeds never tangled with wall-following speeds), we could tune one part without breaking the others. When the course changed, we could update one module in isolation. Past-us tangled these once and present-us paid for it, so: keep your subsystems separate. Lesson very much learned.
 
---
 
## The Strategy (or, How We Decided to Lose on Purpose)
 
I want to be honest about something, because it's my favourite part of how we approached this.
 
By the back half of the term, we knew the truth: **we were not going to win wall-following.** Another group was ripping through the maze in around 1:20, and our maze times just weren't there. Spinning our wheels (sometimes literally) trying to out-maze them was a losing bet.
 
So we made a call. *Lock in on what we're elite at — line-following and object-detection — and play the scoring, not our egos.* My exact pitch to the team:
 
> "we know that wall-following will take us hella long… so lowkey, let's just take the penalty once it gets to the end to reset it so it can ram straight into the cheese — because we can get the best time for both, as long as we finish the maze."
 
The reasoning: since each section was scored, being the *fastest* at line-following and object-detection was worth more to us than being mediocre-everywhere. We even floated a galaxy-brained unifier — *"if you detect a wall on the right and nothing in front, just run straight forward at max speed"* — an algorithm that would rip through open stretches **and** double as object-detection if the cheese was placed dead ahead.
 
Was it the most heroic strategy? No. Was it the *correct* strategy given our constraints? Absolutely. Knowing what to optimize — and what to deliberately not — is its own kind of intelligence. (Very on-brand for a cognitive systems course, if you think about it.)
 
Oh, and a small flex: we were the **only group that got to keep our robot fully intact** after the term, because none of Bob's components were borrowed from the lab. While other teams disassembled their bots for parts, Bob lived. That's so aura.
 
---
 
## Challenges — Explained Per Section
 
Every section of this project tried to break us in its own special way. Here's the honest ledger of what went wrong and what got us through.
 
**🔧 The Build & Wiring**
- *Problem:* Components sliding around mid-run, quietly destroying our wiring and our runs. *Fix:* Properly stabilizing everything with tissue tape and a second chassis deck. Stability was a prerequisite for *any* consistent behaviour.
- *Problem:* The original hammer caster added drag and made turns inconsistent. *Fix:* Swapped in a free-rolling steel ball caster — instantly cleaner turning.
- *Problem:* The eternal battery shortage and voltage drift (turn behaviour literally changed with battery charge level). *Fix:* A dedicated, always-charged battery supply that lived in the lab… in theory.
**📏 Line Following**
- *Problem:* Bob would skip the line on sharp turns, especially that one diabolical hairpin. *Fix:* Tuning the seek-arc — the gap between `LF_SEEK_LEFT` and `LF_SEEK_RIGHT` — so the arc was sharp enough to catch tight turns without overshooting straights. We also added a safeguard: when Bob ran off the line, distance checks against the front/right walls kept him from crashing while he arced back to the tape.
- *The reality check:* At one point we asked, only half-joking, whether we could just physically widen the tape on the hairpin, because a 3-sensor robot is genuinely at the edge of what's trackable on a turn that tight. Sometimes the honest engineering answer is "this is a hardware limitation, not a code bug."
**🧱 Wall Following**
- *Problem:* Getting wedged into corners with frozen sensor readings. *Fix:* The two-tier stuck-recovery system (straight reverse + push, then angled reverse).
- *Problem:* Noisy ultrasonic readings making Bob jittery or panicky. *Fix:* The moving-average filter plus a tolerance dead-band so small fluctuations didn't trigger constant corrections.
- *Problem:* Detecting the *end* of the wall reliably — the far wall took much longer to return a solid reading. *Fix:* Reducing the signal filtering at that moment to boost sensitivity, and only requiring one good "the wall is gone" read to trigger the exit.
**🧀 Object Detection**
- *Problem:* We basically had to build this behaviour twice — our first attempts barely worked and we nearly started from scratch. *Fix:* Instead of starting over, we took robust obstacle-*avoidance* code and inverted it into obstacle-*seeking*. Don't start from zero when you can re-frame something that already works.
- *Problem:* Distinguishing the cheese from the TNT and clock tower. *Fix:* Fusing IR (height off the ground) with an ultrasonic profile sweep (wide vs. narrow) to classify the target.
**🔗 State Transitions**
- *Problem:* This was the hardest thing in the whole project — knowing when one challenge truly ended and the next began, without false triggers from a momentary lost line. *Fix:* A short coast delay plus a confirming condition (e.g., a right wall actually appearing) before committing to a mode switch, and keeping every subsystem's constants modular so we could tune transitions without collateral damage.
**🧠 The Meta-Challenge: Five People, One Robot**
- *Problem:* Coordinating five schedules, one physical robot, and a shared codebase across 12 weeks. *Fix:* Discord as our nerve center, one shared Arduino file as the source of truth ("the code currently burned into the Arduino"), and a hard-won lesson in **git discipline** — `git pull` *before* you push, keep it to one branch, stop creating phantom branches at midnight. We learned version control the way everyone does: by breaking it first.
---
 
## Reflections
 
Safe to say, this went further than any of us expected walking in.
 
Bob is, at his core, a really simple machine — a handful of cheap sensors, two motors, and a few hundred lines of `if` statements. But watching those simple rules add up into *behaviour* — watching Bob lose the line, arc right, find it again, thread a cardboard maze, free himself from a corner, and then go drive straight into a block of cheese — never stopped being a little bit magical. That's the whole lesson of COGS 300, really: intelligence doesn't have to live in some giant brain. A lot of it is just a tight, honest loop between sensing and acting, tuned patiently until it works.
 
There's something almost biological about the failure modes, too. Bob got "stuck." Bob got "confused" at junctions. Bob had reflexes and recovery behaviours. We spent the term anthropomorphizing a robot named Bob, and somewhere along the way we actually internalized how an embodied agent navigates a world it can only partly perceive.
 
And honestly? The robot is only half of it. The other half was five people figuring out how to build something together — splitting up sensors and code, covering for each other when someone had a midterm, debugging at 7am with Tim Hortons timbits and coffee, and turning "Bob skipped the line again" into a running inside joke instead of a crisis. There were absolutely points where it felt like this thing would never come together. But with a lot of twisting, turning, re-tuning, and the occasional crash-out, it did.
 
To Adrian, Ella, Jayson, and Yui — we lowkey cooked. Thank you for the late nights and the aura farming. And to anyone reading this who's about to take COGS 300: you're going to feel, at some point, like you're not "technical enough" to pull this off. You are. We promise. Just keep taking pictures and videos, keep tightening that loop, and trust that the robot will eventually do the thing.
 
Bob lives. 🤖
 
---
 
## Repository Files
 
| File / Folder | What's Inside |
| --- | --- |
| `Final_Version_*` / `Lab_11_12-10.ino` | The final tournament code — the full Line-Follow → Wall-Follow → Exit state machine that ran on demo day. |
| `Robot_Line_Following_Adjustment.ino` | A very consistent line-following build, with the tuned seek-arc constants. |
| `Code_Reversing_Behavior_on_White_Line_Detection.ino` | Line-detection driving logic and behaviour reversal experiments. |
| Earlier `Lab0X_*` files | The week-by-week trail — photocell line-following, ultrasonic wall-following, object detection — basically Bob's fossil record from first wheels to final run. |
| `/drone-shots/` | Overhead drone and stitched panoramas of the full track. |
| `/bob/`, `/general/` | Build photos, run clips, and the moments along the way. |
 
---
 
## References & Citations
 
#### Footnotes
 
1. Brooks, R. A. (1991). *Intelligence without representation.* Artificial Intelligence — the foundational argument that intelligent behaviour can emerge from an agent tightly coupled to its environment, without a central world-model. [↩](#the-course-and-what-autonomous-actually-meant-for-us)
2. Brooks, R. A. (1986). *A robust layered control system for a mobile robot.* IEEE Journal of Robotics and Automation — the subsumption / behaviour-based control approach that our reactive, behaviour-stacked design echoes. [↩](#the-concepts-we-leaned-on)
- Course context: **COGS 300 — Understanding and Designing Cognitive Systems**, University of British Columbia ([course site](https://cogs-300.github.io/)).
- HC-SR04 ultrasonic ranging, L298N dual H-bridge motor driver, and Arduino Uno reference documentation.
- The example obstacle-avoidance sketches we re-framed into object-seeking, and the many late-night Discord threads where this robot was actually, genuinely built.
---
 
*Built over 12 weeks by Adrian, Ella, Jayson, Ridhwan & Yui. Powered by Arduino, cardboard, timbits, and an unreasonable amount of tuning. 🧀*
