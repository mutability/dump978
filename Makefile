CFLAGS+=-O2 -g -Wall -Werror
LIBS=-lm
CC=gcc

all: dump978

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

dump978: dump978.o
	$(CC) -g -o $@ $^ $(LIBS) $(LDFLAGS)

clean:
	rm *.o dump978
