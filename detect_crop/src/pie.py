# -*- coding: utf-8 -*-
"""
Created on Sun Jul 19 18:01:11 2020

@author: taeke
"""

labels = ["flour",
          "ugar",
          "egg",
          "butter",
          "milk",
          "package of yeast"]

data = [225, 90, 50, 60, 100, 5]

text = []
separator = ': '
for label, value in zip(labels, data):
    text.append(label + separator + str(value))
