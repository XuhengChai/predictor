# -*- coding: utf-8 -*-
"""
Created on Tue Oct 10 13:54:15 2023

@author: Z0030883
"""
import math


class UnitConverter():
    kph2mps = 1 / 3.6
    mps2kph = 3.6
    rpm2rps = 1 / 60
    diffrntl_ratio = 2.64
    tyre_radius = 0.51
    deg2rad = math.pi / 180
    tsmo2kph = rpm2rps / (diffrntl_ratio / tyre_radius) * 2 * math.pi * mps2kph
    rpm2kph_wo_radius = rpm2rps * 2 * math.pi * mps2kph
