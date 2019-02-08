#!/usr/bin/env python
# coding: utf8


class Encoder:

    def __init__(self, msg):
        self.msg = msg

    def encode(self):

        ### TO-DO
        ###İleride buraya paket boyutuna göre filtreleme yazılacak
        temp = "{" + self.msg + "}"

        return temp

    def decode(self):

        msg = self.msg.split("{")
        msg1 = msg[1]
        msg2 = msg.split("}")
        return msg2[0]
