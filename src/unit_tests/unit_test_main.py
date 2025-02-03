# -*- coding: utf-8 -*-
"""
Created on Tue Jul  3 20:20:38 2018

@author: Romeo Orsolino
"""

import unittest
loader = unittest.TestLoader()
tests = loader.discover('.')
testRunner = unittest.runner.TextTestRunner()
testRunner.run(tests)


