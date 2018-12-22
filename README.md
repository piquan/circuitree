# Christmas tree circuit

Merry Christmas!  This is a fun little Christmas decoration that I
created as a gift to friends and family: a small circuit board in the
shape of a Christmas tree, with flashing lights and decorative
buttons, and a stand with has an optional nameplate.

![Photo of the first production-complete Christmas tree decoration](https://github.com/piquan/christmas-tree/blob/master/cover-photo.jpg)

I designed and programmed this from scratch for my closest friends and
family.  It’s taken me several months of engineering, but it’s been a
lot of fun, and I learned a lot of new things while I built it.  It’s
conceptually inspired by a similar circuit that Dad and I built in the
80s from a magazine article, but this circuit uses completely
different modern technology.  I sent my blueprints to [Macrofab, a
factory in Houston](https://macrofab.com/) to create the circuit
boards, and to [Jinxbot, a one-man 3D printing company in Mountain
View](https://jinxbot.com/) to create the stands.  I hand-programmed
and signed each circuit board.  In the four months I spent creating
this, I went through three revisions of the circuit board (plus
several smaller test designs), four revisions of the stand, and
hundreds of revisions of the program.  I hope you enjoy the Christmas
lights in your home as much as I’ve enjoyed creating it!

# Contributing

The entire project is open-source; you're encouraged to modify and
improve it.  Many of my friends are makers, so I want to make sure
that they can have extra fun by reading the details.  The project is
controlled by an ATtiny84A, so you can edit and upload the code right
from the Arduino IDE!  The firmware, debugging device, schematics, PCB
design, and decorative artwork is all in GitHub.

If you just want to look at the schematics and don't have KiCAD, you
can go to the "releases" tab in GitHub and download a zip file with
PDFs of the schematic and PCB, Gerber files for the PCB, .hex files
for the firmware, and .stl and .png files of the stand.

For technical documentation, visit the wiki.  There are notes there
about design decisions I've made, how to reprogram the Christmas tree,
and other useful information for makers.

Contributions are very welcome!  The best way to contribute is to open
a pull request or issue on GitHub.

# Credits

The schematic, PCB layout, decorative artwork, and firmware were all
designed by Joel Holveck.

The tlc5947.h LED driver controller library was originally written by
Limor Fried/Ladyada for Adafruit Industries, at
<https://github.com/adafruit/Adafruit_TLC5947/>.  The version included
here has several changes to use less memory, because of the needs of
the ATtiny environment.  I used Adafruit products (such as the
[TLC5947 breakout board](https://www.adafruit.com/product/1429)
extensively during the development of this project, and am very
grateful both for their product line and open-source contributions.

The capsense.h capacitive touch sensor library was inspired by the
library created by Paul Bagder as maintained by Paul Stoffregen, at
<https://github.com/PaulStoffregen/CapacitiveSensor/>.  The
implementation here is completely new, but borrows several ideas from
the Badger/Stoffregen library.  The Badger/Stoffregen capacitive touch
library is a very simple, straightforward capacitive touch sensing
library; seeing the way it was used helped convince me that the design
was viable.

# License

This project is copyright 2018 by Joel Ray Holveck.  All rights
reserved.

Different parts of this project are licensed differently; the licenses
are in the individual directories.  Generally, firmware is under
GPLv3, the PCB is under the CERN OHL, and the decorative artwork is
under CC-BY-SA.
