from ctypes import cdll
import time
lib = cdll.LoadLibrary('./libfoo.so')

class Foo(object):
    def __init__(self):
        self.obj = lib.Foo_new()

    def bar(self):
        lib.Foo_bar(self.obj)

    def end(self):
        lib.Foo_end(self.obj)

f = Foo()

f.bar() #and you will see "Hello" on the screen

f.end()

print('sleeping')
time.sleep(1)

while(True):
    f.end()

