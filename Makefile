CFLAGS+=-O2 -g -Wall -Werror -Ifec
LDFLAGS=
LIBS=-lm
CC=gcc

all: dump978 uat2json

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

dump978: dump978.o uat_decode.o fec/decode_rs_char.o fec/init_rs_char.o
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS)

uat2json: uat2json.o uat_decode.o 
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS)

clean:
	rm *.o fec/*.o dump978 uat2json
