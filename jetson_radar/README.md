# Radar on the Jetson Orin Nano — bring-up (step 1: loss test)

Nothing in here writes radar data to the Jetson. There is no SSD; the receiver
keeps only a few frames in RAM.

## Files
- `jetson_setup.sh`     one-time setup (packages, sysctl, static IP, serial permission, power mode)
- `jetson_check.py`     pre-flight: serial port, network, DCA1000, radar
- `jetson_loss_test.py` the receiver: DCA1000 config + `g` + receive + stats, RAM-only
- `dca1000.py`          DCA1000 control library (imported by the scripts)
- `radar_ctl.py`        manual radar control: `status` / `g` / `s` / `monitor`

## Cabling
- DCA1000 Ethernet  -> Jetson RJ45
- Radar USB (XDS110) -> Jetson USB-A
- Radar 12 V and DCA1000 5 V on their own supplies, as before
- Jetson Wi-Fi for internet/this chat (the RJ45 is dedicated to the DCA1000)

Rule: never power-cycle the DCA1000 while the radar is running. If you must
restart the radar: 12 V out, count to ten, 12 V in, wait 15 s.

## First time
```
cd ~/radar            # wherever you put these files
chmod +x jetson_setup.sh
sudo ./jetson_setup.sh
```
Log out and back in (serial permission), then:
```
python3 jetson_check.py
```
All five lines must say OK. Typical fixes:
- serial "Permission denied" -> you didn't log out/in after setup
- radar silent -> slow power cycle the radar, re-run the check
- DCA1000 no response -> Ethernet cable / DCA1000 power / setup script didn't
  pick the right interface (run `ip link`, then `sudo ./jetson_setup.sh <iface>`)

## The loss test
```
python3 jetson_loss_test.py --seconds 60
```
Every 2 s it prints frames, fps, DCA packets lost in that window, chirps
dropped, and queue depth. At the end: the same summary and PASS/CHECK verdict
as the PC script.

PASS = the Orin Nano takes the full 120 fps / 63 MB/s stream with zero loss.
That is the go/no-go for everything that follows.

If it does not pass, keep the stats lines: `dca_lost` growing with a small
queue means the NIC/kernel is dropping (sysctl/IRQ side); a large queue means
Python is not keeping up (receiver side). Both have known fixes; the numbers
say which.

## Everyday use afterwards
Plug the radar and DCA1000 in whenever. Then:
```
python3 jetson_loss_test.py        # Ctrl+C to stop
python3 radar_ctl.py status        # is the radar alive / what state
```
