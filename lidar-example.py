import ydlidar_x2
import time

port =  'COM13'
lid = ydlidar_x2.YDLidarX2(port)
lid.connect()
lid.start_scan()

try:
    while True:
        if lid.available:
            distances = lid.get_data()
            # process the distance measurements
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

lid.stop_scan()
lid.disconnect()
print("Done")