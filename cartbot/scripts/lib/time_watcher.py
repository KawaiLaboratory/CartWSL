#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

class TimeWatcher:

    start_time = 0
    lap_watch = 0
    lap_pause_time = 0
    pause_start_time = 0
    pause_time = 0
    
    def __init__(self):
        print("Initialization")

    def start(self):
        current_time = time.time()
        if self.start_time == 0:
            self.start_time = current_time
            self.lap_watch = current_time
        else:
            print("Measurement did not finish.")
        
    def pause(self):
        current_time = time.time()
        if self.pause_start_time == 0:
            self.pause_start_time = current_time
        else:
            print("Pause")

    def resume(self):
        current_time = time.time()
        if self.pause_start_time != 0:
            self.pause_time += current_time - self.pause_start_time
            self.lap_pause_time += current_time - self.pause_start_time
            self.pause_start_time = 0
        else:
            print("not pouse")

    def lap(self):
        current_time = time.time()
        lap_time = -1
        if self.pause_start_time == 0:
            lap_time = current_time - self.lap_watch - self.lap_pause_time
            self.lap_watch = current_time
            self.lap_pause_time = 0
        else:
            print("lap - pause")
        return lap_time

    def elapsed(self):
        current_time = time.time()
        r = -1
        if self.pause_start_time == 0:
            r = current_time - self.start_time - self.pause_time
        else:
            print("elapsed - pause")
        return r

    def end(self):
        if self.pause_start_time == 0:
            split_time = self.elapsed()
            lap_time = self.lap()
        else:
            self.resume()
            #resume()後の最新タイムを使う
            split_time = self.elapsed()
            lap_time = self.lap()
        #Initialization
        self.start_time = 0
        self.lap_watch = 0
        self.lap_pause_time = 0
        self.pause_start_time = 0
        self.pause_time = 0

        return split_time,lap_time



