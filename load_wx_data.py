#!/usr/bin/env python

import datetime
import psutil
from influxdb import InfluxDBClient

# influx configuration - edit these
ifuser = "grafana"
ifpass = "JonWb2mnf"
ifdb   = "home"
ifhost = "127.0.0.1"
ifport = 8086
measurement_name = "system"

# take a timestamp for this measurement
time = datetime.datetime.utcnow()

# collect some stats from psutil
disk = psutil.disk_usage('/')
mem = psutil.virtual_memory()
load = psutil.getloadavg()

# format the data as a single measurement for influx
body = [
    {
        "measurement": measurement_name,
        "time": time,
        "fields": {
            "load_1": load[0],
            "load_5": load[1],
            "load_15": load[2],
            "disk_percent": disk.percent,
            "disk_free": disk.free,
            "disk_used": disk.used,
            "mem_percent": mem.percent,
            "mem_free": mem.free,
            "mem_used": mem.used
        }
    }
]

# connect to influx
ifclient = InfluxDBClient(ifhost,ifport,ifuser,ifpass,ifdb)

# write the measurement
ifclient.write_points(body)



