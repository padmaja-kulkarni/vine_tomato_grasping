# -*- coding: utf-8 -*-
"""
Created on Sat Jul 18 21:24:47 2020

@author: taeke
"""


# taken from https://realpython.com/python-timer/
# modifief such that it works with python 2!

import time
import functools

class TimerError(Exception):
    """A custom exception used to report errors in use of Timer class"""


class Timer:
    timers = dict()

    def __init__(
        self,
        name=None,
        text="Elapsed time: {:0.4f} seconds",
        logger=False,
    ):
        self._start_time = None
        self.name = name
        self.text = text
        self.logger = logger

        # Add new named timers to dictionary of timers
        if name:
            self.timers.setdefault(name, 0)

    def start(self):
        """Start a new timer"""
        if self._start_time is not None:
            raise TimerError("Timer is running. Use .stop() to stop it")
        
        self._start_time = time.time() #perf_counter()

    def stop(self):
        """Stop the timer, and report the elapsed time"""
        if self._start_time is None:
            raise TimerError("Timer is not running. Use .start() to start it")

        elapsed_time = time.time() - self._start_time # perf_counter()
        self._start_time = None

        if self.logger:
            print(self.text.format(elapsed_time))
        if self.name:
            self.timers[self.name] += elapsed_time

        return elapsed_time
        
    def __enter__(self):
        """Start a new timer as a context manager"""
        self.start()
        return self

    def __exit__(self, *exc_info):
        """Stop the context manager timer"""
        self.stop()
        
    def __call__(self, func):
        """Support using Timer as a decorator"""
        @functools.wraps(func)
        def wrapper_timer(*args, **kwargs):
            with self:
                return func(*args, **kwargs)
    
        return wrapper_timer
        
@Timer("sleep", text="Downloaded the tutorial in {:.2f} seconds")
def cust_sleep(sleep_time):
    time.sleep(sleep_time)
        
if __name__ == '__main__':
    t = Timer("sleep", logger=True)
    
    with t:
        time.sleep(0.2)
    
    cust_sleep(0.5)
    
    
    print(Timer.timers)