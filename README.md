# Pelmetcam

Martin O'Hanlon [martin@ohanlonweb.com](mailto:martin@ohanlonweb.com)

[Stuff About Code](http://www.stuffaboutcode.com)

[Software for my Raspberry pi power Helmet cam](http://www.stuffaboutcode.com/2014/01/raspberry-pi-gps-helmet-cam.html)

## Version history

0.1 - first alpha release
0.x - 202x code refactoring (work in progress)

## Notes

04/01/2013
Due to an [error with the rpi kernel](https://github.com/raspberrypi/linux/issues/435) you need to use kernel version Linux picam 3.6.11+ #557, until the bug is fixed downgrade kernel with command sudo rpi-update 8234d5148aded657760e9ecd622f324d140ae891

## Modules needed:

- PIL
- picamera
- MP4Box, mencoder (encoding videos)
- GPSd

```bash
apt install -y --no-install-recommends gpsd gpsd-clients python3-gps python3-pil python3-picamera2 mencoder ffmpeg
```
