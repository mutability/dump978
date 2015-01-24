# dump978

Experimental demodulator/decoder for 978MHz UAT signals.

This expects 8-bit I/Q samples on stdin at 2.083334MHz, for example:

````
$ rtl_sdr -f 978000000 -s 2083334 -g 48 - | ./dump978
````
