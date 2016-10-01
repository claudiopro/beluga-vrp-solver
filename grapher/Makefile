SRCROOT=..
LIBRARIES=/usr/local/lib/libglpk.a /usr/local/lib/concorde.a /usr/local/lib/qsopt.a
CFLAGS=-O2
DEBUGFLAGS=-DDEBUG

default: grapher

grapher: grapher.c grapher.h $(SRCROOT)/datautils.c
	$(CC) -o $@ $(CFLAGS) $(SRCROOT)/datautils.c $@.c $(LIBRARIES)

debug: grapher.c $(SRCROOT)/datautils.c
	$(CC) -o grapher $(DEBUGFLAGS) $(SRCROOT)/datautils.c grapher.c $(LIBRARIES)

clean:
	rm -rf *.o *.exe
