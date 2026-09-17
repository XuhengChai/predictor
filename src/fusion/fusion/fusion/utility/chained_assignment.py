# -*- coding: utf-8 -*-
"""
Created on Sun Apr  9 10:54:36 2023

@author: https://gist.github.com/notbanker
"""

import pandas as pd


class ChainedAssignment:
    """ Context manager to temporarily set pandas chained assignment warning.
        Usage:

        with ChainedAssignment():
             blah

        with ChainedAssignment('error'):
             run my code and figure out which line causes the error!

    """

    def __init__(self, chained=None):
        acceptable = [None, 'warn', 'raise']
        assert chained in acceptable, "chained must be in " + str(acceptable)
        self.swcw = chained

    def __enter__(self):
        self.saved_swcw = pd.options.mode.chained_assignment
        pd.options.mode.chained_assignment = self.swcw
        return self

    def __exit__(self, *args):
        pd.options.mode.chained_assignment = self.saved_swcw
