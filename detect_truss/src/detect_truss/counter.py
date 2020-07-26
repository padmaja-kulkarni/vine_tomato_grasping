# -*- coding: utf-8 -*-
"""
Created on Mon Jul 20 10:49:03 2020

@author: taeke
"""

# -*- coding: utf-8 -*-
"""
Created on Sat Jul 18 21:24:47 2020

@author: taeke
"""


import functools

class CounterError(Exception):
    """A custom exception used to report errors in use of Counter class"""


class Counter:
    counters = dict()

    def __init__(
        self,
        name='count',
        name_space = None,
        text="Total calls: {:0.4f}",
        logger=False,
    ):
        self.name = name
        self.text = text
        self.logger = logger
        self.name_space = name_space

        # Add new named counter to dictionary of counters
        val = 0

        if self.name_space:
            self.counters.setdefault(self.name_space, {})
            self.counters[self.name_space].setdefault(self.name, val)
        else:
            self.counters.setdefault(name, val)

    def start(self):
        """Start a new counter"""
        if self.name_space:
            d = self.counters[self.name_space]
        else:
            d = self.counters
            
        d[self.name] += 1

    def stop(self):
        """Stop the counter, and report the counts"""
        
            
        if self.name_space:
            d = self.counters[self.name_space]
        else:
            d = self.counters        
            
        count = d[self.name]
        if self.logger:
            print(self.text.format(count))

        return count
        
    def __enter__(self):
        """Start a new counter as a context manager"""
        self.start()
        return self

    def __exit__(self, *exc_info):
        """Stop the context manager counter"""
        self.stop()
        
    def __call__(self, func):
        """Support using Counter as a decorator"""
        @functools.wraps(func)
        def wrapper_counter(*args, **kwargs):
            with self:
                return func(*args, **kwargs)
    
        return wrapper_counter
        
#@Timer("sleep", text="Downloaded the tutorial in {:.2f} seconds")
#def cust_sleep(sleep_time):
#    time.sleep(sleep_time)
#        
#if __name__ == '__main__':
#    t = Timer("sleep", logger=True)
#    
#    with t:
#        time.sleep(0.2)
#    
#    cust_sleep(0.5)
#    
#    
#    print(Timer.timers)