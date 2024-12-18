MAIN_DTS_FILE := orange-pi-3b-picnic
DTS_CPP_FLAGS := -nostdinc -Iinclude -undef -D__DTS__ -x assembler-with-cpp

all: ${MAIN_DTS_FILE}.dtbo clean-deps

${MAIN_DTS_FILE}.dts.tmp:
	cpp -Wp,-MD,${MAIN_DTS_FILE}.dts.dep.tmp ${DTS_CPP_FLAGS} -o ${MAIN_DTS_FILE}.dts.tmp ${MAIN_DTS_FILE}.dts

${MAIN_DTS_FILE}.dtbo: ${MAIN_DTS_FILE}.dts.tmp
	dtc -I dts -O dtb -o ${MAIN_DTS_FILE}.dtbo -b 0 -d ${MAIN_DTS_FILE}.dts.dep.tmp ${MAIN_DTS_FILE}.dts.tmp

clean-deps:
	rm -f *.dep.tmp
	rm -f *.dts.tmp

install:
	sudo armbian-add-overlay ${MAIN_DTS_FILE}.dts
