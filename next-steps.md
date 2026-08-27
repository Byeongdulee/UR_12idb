# 12idUR Next Steps

## Current Status

The pyDevSup robot connection path is working for the test robot `BC`.

The successful runtime sequence was:

```text
Connect -> Connecting
remote-control preflight passed
UR3("BC") constructor returned
Connecting -> Connected
ConnectionError remained empty
```

`12idUR:Connect` is a momentary command. It returns to `Idle` immediately while one shared worker performs robot construction. Connection progress is reported separately through:

- `12idUR:ConnectionState`
  - `0`: Disconnected
  - `1`: Connecting
  - `2`: Connected
  - `3`: Error
- `12idUR:ConnectionError`

Before constructing the Python robot, the worker checks:

```text
12idUR:Dashboard:IsInRemoteControl.RVAL
```

The connection fails without opening Python robot sockets if that record is zero, undefined, or alarmed.

## Retained Fixes

- Robot construction runs outside the record-processing thread.
- Only one robot worker operation can run at a time.
- Connection status is published with `IOScanListThread`.
- TCP setup waits for fresh secondary-monitor packets instead of spinning on cached data.
- TCP verification has a two-second timeout and strict component tolerance.
- A failed TCP setup closes the partially constructed robot before propagating the error.
- Secondary-monitor program transmission uses `sendall()` and reports queue timeout or socket failure.
- IOC startup remains disconnected; the robot is constructed only after `12idUR:Connect` is processed.

## Test Configuration Warning

`BC` is currently mapped to `164.54.104.148` in `list_of_robots.json`.

For testing, `ini/UR3.ini` and `ini/UR3path.ini` were copied to:

```text
ini/BC.ini
ini/BCpath.ini
```

These copied files are not independently validated BC calibration. Do not treat the successful connection as validation of motion geometry, waypoints, sample positions, or paths. Before any motion test, either validate the BC copies explicitly or separate robot endpoint identity from calibration-profile selection in `UR3`.

## Next Steps

1. Verify connection PV behavior with Channel Access:

   ```sh
   camonitor 12idUR:ConnectionState 12idUR:ConnectionError
   caput 12idUR:Connect 1
   ```

   Confirm `Connecting -> Connected`, an empty error, and rejection of a second connect request.

2. Decide how robot identity and calibration identity should be modeled.

   Preferred direction: allow the endpoint name `BC` to use an explicitly selected calibration profile instead of maintaining copied INI files. Do not implement an implicit fallback from missing `BC.ini` to `UR3.ini`.

3. Add an explicit disconnect command.

   It should run on the same serialized worker, close robot/camera/dashboard/monitor resources, clear the shared robot reference, and publish `Disconnected`. IOC shutdown should use the same cleanup path.

4. Reduce the remaining verbose debug output in `robot_connection.py` after connection and disconnect behavior are stable. Keep concise error logging and state-transition logging.

5. Define the first bounded IOC robot primitives.

   Start with reviewed operations that have clear limits and no arbitrary pose input. Every operation must reject requests unless connected, verify remote-control status, use the shared worker, and reject overlapping commands.

6. Add command state and completion identifiers before exposing hardware operations.

   Clients must not infer operation completion from a momentary command record. Publish an operation identifier, state, and error/result fields.

7. Test failure paths without motion:

   - Dashboard disconnected or status invalid.
   - Teach pendant in local control mode.
   - Duplicate connect request.
   - TCP mismatch or TCP command timeout.
   - Missing calibration files.
   - IOC shutdown while connected.

8. Review the retained low-level changes in:

   ```text
   common/robUR.py
   urxe/robot.py
   urxe/ursecmon.py
   ```

   Keep them if the project wants strict TCP verification and acknowledged program transmission globally. Add focused non-hardware tests where practical.

9. Before the first physical command test, confirm the robot identity, active TCP, payload, attached gripper, validated BC calibration, clear workspace, reduced speed, and an operator at the emergency stop.

## Validation So Far

- Python syntax parsing passed for the changed Python files.
- `git diff --check` passed in the root and nested `12idUR` repositories.
- Runtime connection to `BC` reached `ConnectionState=Connected`.
- No robot motion, gripper operation, camera alignment, or path execution has been validated.
