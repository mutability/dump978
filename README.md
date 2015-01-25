# dump978

Experimental demodulator/decoder for 978MHz UAT signals.

This expects 8-bit I/Q samples on stdin at 2.083334MHz, for example:

````
$ rtl_sdr -f 978000000 -s 2083334 -g 48 - | ./dump978
````

Run with no args to get verbose decoding.
Run with -raw to get raw message data only.

See sample-output.txt for some example output.

## uat2json

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
  ./dump978 -raw | \
  ./uat2json /var/www/dump978map/data
````

5) Go look at http://localhost/dump978map/
