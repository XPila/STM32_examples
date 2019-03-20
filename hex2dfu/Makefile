#-------------------------------------------------------------------------------
# Makefile to compile and link hex2dfu

TARGET = hex2dfu

CCSRC = hex2dfu.c ed25519/sha512.c ed25519/keypair.c ed25519/sign.c ed25519/ge.c\
 ed25519/add_scalar.c ed25519/sc.c ed25519/verify.c ed25519/fe.c

CCFLAGS = -g -Os -w -Wall -std=gnu99 -MMD
CCFLAGS += -DF_CPU=$(F_CPU)UL -DBOARD=$(BOARD)
CCFLAGS += -D__MSYS__
CCFLAGS += -I'/c/MinGW/msys/1.0/include'

LDFLAGS = -g -Os -w
LDFLAGS += -L'/c/MinGW/msys/1.0/lib'
LDFLAGS += -L'/c/MinGW/msys/1.0/lib/w32api'
LDFLAGS += -l msys-1.0.dll


CC = /c/MinGW/msys/1.0/bin/gcc.exe

OBJ = $(CCSRC:.c=.o)


build: exe

exe: $(TARGET).exe


clean:
	rm -f $(TARGET).exe
	rm -f $(TARGET).map
	rm -f $(CCSRC:.c=.o)
	rm -f $(CCSRC:.c=.d)

.SECONDARY : $(OBJ)
%.o : %.c
	@echo
	@echo 'Compilig' $<
	$(CC) -c $(CCFLAGS) $< -o $@ 

.SECONDARY : $(TARGET).exe
%.exe: $(OBJ)
	@echo
	@echo 'Linking' $@
	$(CC) $(CCFLAGS) $^ --output $@ $(LDFLAGS)
