#!/usr/bin/env python
# -*- coding:utf-8 -*-

import utils.gnss_converter as gc

def main():
    map_waypoint = [[37.4483338, 126.6537511, 50.0],[37.4487297, 126.6539784, 50.0], [37.4483338, 126.6537511, 49.0]]
    waypoint_check = []
    
    for waypoint in map_waypoint:
            n,e,_ = gc.enu_convert(waypoint)
            waypoint_check.append([n,e])

    print(waypoint_check)

if __name__ == '__main__':
    main()



