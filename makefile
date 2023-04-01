#FlexProp
PORT = /dev/ttyUSB0
FLEXCC = ~/flexprop/bin/flexcc.mac
LOAD = ~/flexprop/bin/loadp2.mac
FLEXCCFLAGS=-2 -O1 -Wall -D_DEBUG_INFO -D_DEBUG_WARNING -D_DEBUG_ERROR#-D_DEBUG #-g
BIN = bin

FIND := find #C:/cygwin64/bin/find.exe

TARGET = FRED.binary
SRC = . #flexprop/include # Root folders for src code
INCLUDE = $(sort $(dir $(foreach dir,$(SRC),$(shell $(FIND) $(dir) -name "*.h" -o -name "*.spin2"))))
SOURCE = $(sort $(foreach dir,$(SRC),$(shell $(FIND) $(dir) -name "*.c")))
SPIN = $(sort $(foreach dir,$(SRC),$(shell $(FIND) $(dir) -name "*.spin2")))

OBJECTS := $(addprefix $(BIN)/,$(SOURCE:.c=.o))
OBJECTSPIN := $(addprefix $(WIN)/,$(SPIN:.spin2=.h))
INC := $(addprefix -I ,$(INCLUDE))

flexpropc: clean clear flexprop
flexprop: $(BIN)/$(TARGET) run

# Make for FlexCC
$(BIN)/$(TARGET): $(OBJECTS)
	$(FLEXCC) $(FLEXCCFLAGS) $(INC) $(OBJECTS) -o $@

$(BIN)/%.o: %.c
	-mkdir -p $(subst \,/,$(@D))
	$(FLEXCC) $(FLEXCCFLAGS) $(INC) -c -o $@ $<
	
run: 
	$(LOAD) -b230400 -t -ZERO -v $(subst \,/,bin\$(TARGET)) 
.PHONY: clean
.PHONY: clear
clear:
	clear
clean:
	rm -r $(BIN) || true
	-mkdir -p $(BIN)