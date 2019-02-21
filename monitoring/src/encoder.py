#!/usr/bin/env python
# coding: utf8


class Encoder:

    def __init__(self, msg):
        self.msg = msg

    def encode(self):

        ### TO-DO
        ###İleride buraya paket boyutuna göre filtreleme yazılacak
        temp = '{' + self.msg + '}'

        return temp

    
