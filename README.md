# dump978

Experimental demodulator/decoder for 978MHz UAT signals.

## Demodulator

dump978 is the demodulator. It expects 8-bit I/Q samples on stdin at
2.083334MHz, for example:

````
$ rtl_sdr -f 978000000 -s 2083334 -g 48 - | ./dump978
````

It outputs one one line per demodulated message, in the form:

````
+012345678..; this is an uplink message
-012345678..; this is a downlink message
````

For parsers: ignore everything between the first semicolon and newline that
you don't understand, it will be used for metadata later. See reader.[ch] for
a reference implementation.

## Decoder

To decode downlink messages into a readable form (uplink messages are
not yet handled) use uat2text:

````
$ rtl_sdr -f 978000000 -s 2083334 -g 48 - | ./dump978 | ./uat2text
````

See sample-output.txt for some example output.

## Map generation via uat2json

uat2json writes aircraft.json files in the format expected by dump1090's
map html/javascript.

To set up a live map feed:

1) Get a copy of dump1090, we're going to reuse its mapping html/javascript:

````
$ git clone https://github.com/mutability/dump1090 dump1090-copy
````

2) Put the html/javascript somewhere your webserver can reach:

````
$ mkdir /var/www/dump978map
$ cp -a dump1090-copy/public_html/* /var/www/dump978map/
````

3) Create an empty "data" subdirectory

````
$ mkdir /var/www/dump978map/data
````

4) Feed uat2json from dump978:

````
$ rtl_sdr -f 978000000 -s 2083334 -g 48 - | \
  ./dump978 | \
  ./uat2json /var/www/dump978map/data
````

5) Go look at http://localhost/dump978map/
