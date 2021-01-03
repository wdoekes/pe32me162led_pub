pe32me162led_pub
================

Project Energy 32: *Count LED pulses from ISKRA ME-162, export to MQTT*

This project contains ESP8266 code to read LED pulse counts from the
*ISKRA ME-162 electricity meter* and push calculated Watt usage to an
MQTT broker.


HOWTO
-----

Attach an analog light sensor to the ESP8266. Configure and upload this
sketch, as described in the `pe32me162led_pub.ino
<pe32me162led_pub.ino>`_ source code.

This is a *quick and dirty* version of `pe32me162ir_pub
<https://github.com/wdoekes/pe32me162ir_pub>`_. *That version* gets actual
totals from the meter using a serial protocol. *This version* simply
counts pulses from the blinking LED.

I've been told that this version stops giving reliable results once you
start producing energy (using solar panels), because *the same* LED
would blink for both produced and consumed Watt hours.

Having said that, if you don't have an optical probe (infrared
transceiver) to talk to your meter, this version gives surprisingly
accurate results.

*Note that setting up a MQTT broker and a subscriber for the pushed data
is beyond the scope of this HOWTO. Personally, I use Mosquitto (broker),
a custom subscriber, PostgreSQL (with timescale) and Grafana for
visualisation.*
