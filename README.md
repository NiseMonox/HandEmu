# HandEmu

A SteamVR driver that turns Apple Vision Pro hand tracking, streamed by KRVR, into emulated controllers. With it, games that only understand controllers can be played with bare hands. Games see Oculus Touch controllers, and a gesture-oriented binding is included for games that use SteamVR Input.

Windows · SteamVR · current version 1.0.4

## Why

- **KRVR does not emulate controllers.** It gives SteamVR one `krvr_hand` device per hand: a hand skeleton, a pinch trigger and a system gesture. That is deliberate. Hand tracking only works in games that read hands directly, such as VRChat.
- **Controller-only games cannot use those devices.** Many games use SteamVR Input and ship bindings only for controllers (Vive, Touch, Index, WMR). With `krvr_hand` they have no binding, so the hands don't even get a position.
- **Existing tools don't fit.** Leap Motion based integrations need Leap hardware, and ALVR's gesture emulation only works when streaming through ALVR. A custom binding for `krvr_hand` would expose a single button.

## How it works

```
Vision Pro --KRVR--> krvr_hand devices in SteamVR (skeleton + pose + system gesture)
                           |  HandEmu intercepts them inside vrserver
                           v
                 gesture recognition (pinch / fist / drag)
                           v
        two emulated handemu_hand controllers (Oculus Touch button layout)
             |- SteamVR Input games: a binding designed for hand gestures
             '- other games: Oculus Touch compatibility mode
```

- SteamVR has no API that lets one driver read another driver's input. HandEmu therefore hooks `IVRServerDriverHost::TrackedDevicePoseUpdated` and a few `IVRDriverInput` calls inside vrserver, the same technique [OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) uses. It then mirrors what KRVR sends onto its own devices.
- The emulated controllers use a hand selection priority of 100 and KRVR's hands use -1, so games pick the emulated controllers.
- Hand poses are forwarded unchanged. Finger skeletons are rebuilt in SteamVR's conventions; see [Finger skeleton](#finger-skeleton).

## Gestures

| Gesture | Emulated input | Typical use |
|---|---|---|
| Thumb + index **pinch** | Trigger | Click UI and select what you point at. In games that support it, pinch in empty space and drag to move the world (turn the wrist to rotate) |
| Thumb **near** the index tip | Trigger touch | Show the pointer |
| **Fist** | Grip | Grab, up close or with the laser |
| Thumb + middle pinch | A / X | Open or close the game menu |
| Thumb + pinky pinch | B / Y | Secondary action, such as toggling a held grab |
| Thumb + ring pinch, **then move the hand** | Joystick | Free move relative to where you look (about 6 cm of travel is full deflection) |
| **Tap the other palm with an index fingertip** (in and out within 1 s) | System | Open or close the SteamVR dashboard |
| **Press the palms together and hold for 1.5 s** | — | Realign the hands after a Digital Crown recentre (see below) |

The game decides what a pinch does. For example, a game may not drag the world while the laser points at UI.

In the **SteamVR dashboard**:
- index pinch: click;
- ring pinch and drag: scroll;
- middle pinch: home;
- pinky pinch: back.

**Other SteamVR games** see Oculus Touch controllers:

| Touch input | Gesture |
|---|---|
| Trigger | Index pinch |
| Grip | Fist |
| A / X | Middle pinch |
| B / Y | Pinky pinch |
| Joystick | Ring pinch and drag |

Any game can be remapped in SteamVR's controller bindings.

## Finger skeleton

Some games animate their hands from the SteamVR skeleton. They take each finger bone's rotation relative to SteamVR's reference hand (the skeleton Valve Index controllers use) and split it into bend, spread and twist angles. That only works when every bone frame follows SteamVR's conventions: bones run along +X on the left hand and -X on the right, and fingers curl about +Z.

KRVR's joint positions are accurate, but its bone frames do not follow that layout, the left hand least of all. Fed straight into such a game, the fingers bend the wrong way and the left thumb flips back and forth. HandEmu therefore uses only the joint positions and rebuilds both hands in SteamVR's conventions. Each bone starts from the reference pose and is turned onto the next joint by the smallest possible rotation. The joints stay exactly where KRVR reported them, and the bends land on the axes games expect.

SteamVR skeletons come in two motion ranges: "with controller" (fingers wrap around a controller) and "without controller" (full range). A game that reads the "with controller" range gets a hand that only closes to a half fist. There is no controller in your hand, so HandEmu sends the full-range data for both ranges by default.

If a fist still looks too open in game, raise `fingerCurlGain`; see [Configuration](#configuration). It can be changed while SteamVR is running.

## Hand misalignment after a Digital Crown recentre

Pressing and holding the Vision Pro's Digital Crown recentres the view. KRVR keeps the headset continuous, but the hands are then sent turned and shifted relative to the head. On test hardware the shift was 30.7° and 59 cm, so every hand in SteamVR and in game ended up off target. KRVR notices this (its log says `frames turned apart - Crown recentre`) but does not correct it.

HandEmu **only records** such a jump in the background and never changes anything on its own. A jump is recorded when:
- both hands leap at the same instant by the same horizontal rotation and shift, without changing height, while the head stays still; or
- one hand is out of view (for example, the one pressing the Crown) and the other makes an instant, purely horizontal turn of at least 3°.

When the hands feel off, **press your palms together and hold for 1.5 s**, and the recorded jumps are undone exactly.

- Several recentres in a row are undone together. The log shows `Hand frame jump recorded` and `Calibration gesture: undid ...`.
- The gesture checks whether the finger joints of the two hands meet (the two joints after the index, middle and ring knuckles) and whether the middle knuckles are close. It does not look at the wrists or palm centres, because Vision Pro tends to report the wrists 9–10 cm apart in this pose.
- Short tracking dropouts (up to 0.3 s) do not restart the hold. The log shows `Palms together ...`, and `Palms apart again after ... s` if the hands separate too early.
- A recentre that happens while **both** hands are out of view, or before SteamVR started, cannot be recorded. Calibration then reports `no Crown recentre recorded`; reconnect KRVR, which starts both spaces aligned again.
- Recorded jumps are cleared when the headset disconnects.

## Requirements

- Windows 10 or 11 with SteamVR.
- Apple Vision Pro streaming through KRVR with hand tracking on (input source Auto or Hands).
- To build: Visual Studio 2022 or newer with the "Desktop development with C++" workload.

## Installation

1. Get the driver in one of two ways:
   - Download `HandEmu-<version>.zip` from the [Releases](../../releases) page and unzip it somewhere permanent.
   - Build it by running `build.bat`, which writes `driver\handemu\bin\win64\driver_handemu.dll`.
2. Close SteamVR and run `install.bat`. This registers the `driver\handemu` folder with SteamVR (through `vrpathreg`), so leave the folder where it is.
3. Start SteamVR through KRVR as usual, then start your game.

To uninstall, close SteamVR and run `uninstall.bat`.

To switch the driver off temporarily, use SteamVR → Settings → Startup / Shutdown → Manage Add-ons and turn off `handemu`. Do this when using real controllers such as PSVR2. Otherwise the emulated controllers take the hand roles.

### Checking that it works

Open SteamVR's server log (`<Steam>\logs\vrserver.txt`) and search for `handemu:`. A working setup logs:

```
handemu: HandEmu 1.0.4 starting; hand-tracking source type 'krvr_hand'
handemu: Hooks installed; waiting for hand-tracking devices
handemu: Skeleton: rebuilt in SteamVR conventions, full range for both motion ranges, finger gain 1.00, thumb gain 1.00
handemu: Debug folder: ...\driver\handemu\debug (request.txt = capture, live.txt = live skeleton tuning)
handemu: Found left hand-tracking source (device 3, type krvr_hand)
handemu: Hooked left source skeleton
handemu: Hooked left source system button
handemu: Emulated left controller active (device 5)
```

Each "left" line has a matching "right" line.

| What the log shows | Cause and fix |
|---|---|
| No `handemu:` lines at all | The driver is not loaded. Check SteamVR → Settings → Startup / Shutdown → Manage Add-ons |
| `Could not hook vrserver` | Hooking failed, probably because SteamVR changed its internal interfaces |
| `starting`, but no `Hooked ... source skeleton` | No KRVR hands. Make sure KRVR is in hand-tracking mode |
| `Emulated ... controller active`, but nothing happens in game | No binding is applied for the game. In SteamVR → Settings → Controllers → Manage Controller Bindings, pick the game and choose a HandEmu binding, or remap it yourself |

## Configuration

Defaults live in `driver\handemu\resources\settings\default.vrsettings`. To override them, close SteamVR and add a `driver_handemu` section to `<Steam>\config\steamvr.vrsettings`, listing only the keys you change:

```json
"driver_handemu" : {
    "debugLog" : true,
    "pinchOn" : 0.020
}
```

| Key | Default | Meaning |
|---|---|---|
| `pinchOn` / `pinchOff` | 0.012 / 0.020 | Fingertip distance (m) below which a pinch starts, and above which it ends. KRVR reports 1–5 mm for a firm pinch |
| `nearOn` / `nearOff` | 0.035 / 0.045 | "About to pinch" distance, reported as trigger touch (games often show their pointer on it). Spread the two values apart if the pointer flickers |
| `fingerMargin` | 0.008 | How much closer than the index a middle, ring or pinky fingertip must be to count as that finger's pinch |
| `fistOn` / `fistOff` | 0.70 / 0.55 | Fist thresholds on the average curl (0–1) of the middle, ring and pinky fingers |
| `fistIndexOn` / `fistThumbNear` | 0.25 / 0.045 | A fist also needs the index curled at least this much, or the thumb within this distance (m) of the index tip. Tracking often reports a straight index inside a fist |
| `fistFrames` | 4 | Consecutive frames before a fist registers |
| `smoothing` | 0.5 | Weight of the newest sample (1 = no smoothing) |
| `joystickEnable` | true | When off, a ring pinch becomes B / Y |
| `joystickRadius` / `joystickDeadzone` | 0.06 / 0.15 | Hand travel (m) for full joystick deflection, and the dead zone |
| `palmTapEnable` / `palmTapDistance` | true / 0.035 | Index-taps-palm opens the SteamVR dashboard; how close (m) the fingertip must get |
| `calibrationGesture` / `calibrationHoldTime` | true / 1.5 | Palms-together calibration, and how long (s) to hold it |
| `skeletonMode` | 1 | 1 = rebuild finger skeletons in SteamVR conventions, 0 = forward KRVR's skeletons unchanged |
| `skeletonFullRange` | true | Send the full-range (without controller) skeleton for both motion ranges |
| `fingerCurlGain` / `thumbCurlGain` | 1.0 / 1.0 | Scale finger / outer thumb joint curl. Try 1.2–1.4 if a fist does not close fully in game |
| `maxBend` | 115 | Upper limit (degrees) for a scaled joint |
| `logBends` | false | Log every joint's bend angle (KRVR's and the one sent) once a second |
| `posOffsetX/Y/Z`, `rotOffsetX/Y/Z` | 0 | Move or rotate the emulated controllers (m / degrees, right hand; mirrored for the left) |
| `forwardSystemButton` | true | Forward KRVR's own system gesture |
| `sourceControllerType` | krvr_hand | Which devices to read hands from |
| `debugLog` | true | Verbose logging. It logs:<ul><li>every pinch, fist and drag as it starts and ends;</li><li>each hand's measurements once a second;</li><li>data rates every 5 s.</li></ul>It is on while the driver is being tuned |

### Live tuning and captures without restarting SteamVR

The running driver watches a `debug` folder next to its resources (`driver\handemu\debug`):

- **`live.txt`**: `key = value` lines for the skeleton keys above (`skeletonMode` through `logBends`). Changes apply within a second and are logged as `Live tuning: ...`. Delete the file to return to the defaults.
- **`request.txt`**: write `name seconds` (for example `fist 3`). For that many seconds, the driver records KRVR's raw skeletons, the skeletons it sends and the hand poses into `name.csv`, then deletes the request.

### In game

If the laser does not line up with the hand, adjust the game's pointer angle setting if it has one. If the hand model sits off the real hand, adjust the game's hand offset settings, or move the emulated controllers with `posOffset` / `rotOffset` above.

## Building and tests

- `build.bat [output folder]` builds the driver with the Visual Studio toolchain found by `vswhere`.
- `build_tests.bat` builds into `build\stage` and runs two test programs. The staged DLL is separate from the installed one, so this works while SteamVR runs:
  - `tests\skeleton_test.cpp`: unit checks for the skeleton rebuild. The reference and curled hands must come back unchanged, the same hand in arbitrary bone conventions must give the same result, and curl gains are checked.
  - `tests\mock_vrserver.cpp`: a fake vrserver that loads the driver DLL and plays the part of KRVR, then checks what the emulated controllers report. It covers:
    - gestures and pose forwarding;
    - the skeleton rebuild;
    - captures and live tuning;
    - palm taps and calibration, including a palms-together pose measured on hardware;
    - recording and undoing recentre jumps;
    - releasing buttons when tracking is lost;
    - shutdown.
- vrserver keeps the installed DLL locked, so close SteamVR before replacing `driver\handemu\bin\win64\driver_handemu.dll` with a new build.

## Status

Verified with a Vision Pro on SteamVR, in the dashboard and in a SteamVR Input game:
- data flow and hand roles;
- pinch, fist and joystick input;
- palm tap opening the dashboard;
- recentre jump recording.

The finger skeleton rebuild and the finger-joint palms-together detection are new in 1.0.4. So far they have been tested only against synthetic data.

## Known limitations

- HandEmu hooks `IVRServerDriverHost_006` and `IVRDriverInput_004`, the interface versions current SteamVR and KRVR use. If either changes them, gestures stop working and the log says so.
- When a hand leaves view or is hidden for more than 250 ms, all its buttons are released.
- Middle, ring and pinky pinches are easier to misread than the index pinch; tune `pinchOn` and `fingerMargin` if needed. Avoid making a fist and pinching at the same time.
- Vision Pro system gestures can still fire. KRVR's gesture lock can turn them off.
- vrserver sometimes hangs on shutdown. This happens without HandEmu too: the log stops in KRVR's cleanup, and Steam ends the old process on the next start.

## Project layout

```
install.bat / uninstall.bat / install.ps1   register / unregister the driver with SteamVR
build.bat                                   build the driver
build_tests.bat                             build, then run the tests
src\driver.cpp                              hooks, emulated controllers, device handling
src\gestures.cpp / gestures.h               gesture recognition
src\skeleton.cpp / skeleton.h               finger skeleton rebuild in SteamVR conventions
src\capture.cpp / capture.h                 debug folder: captures and live tuning
src\vrmath.h                                vector / quaternion helpers
tests\                                      fake vrserver and skeleton unit tests
driver\handemu\                             the folder SteamVR loads
  resources\input\                          controller profile, gesture / dashboard / legacy bindings
  resources\rendermodels\                   pose-only render models
  resources\settings\default.vrsettings     default settings
third_party\                                OpenVR driver header and MinHook (see third_party\README.md)
```

## Credits

- [MinHook](https://github.com/TsudaKageyu/minhook) by Tsuda Kageyu (BSD 2-Clause) and the [OpenVR SDK](https://github.com/ValveSoftware/openvr) driver header by Valve (BSD 3-Clause), both bundled in `third_party`.
- The hooking approach follows [OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator).

HandEmu is not affiliated with Apple, Valve or KRVR. Product names belong to their owners.

## License

MIT, see [LICENSE](LICENSE). The bundled third-party code keeps its own licenses (see [third_party/README.md](third_party/README.md)).
