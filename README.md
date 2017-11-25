CGgminer v4.6.1 - HexMiner
==============

CGMiner v4.6.1 with Hexminer ASIC support.

This file describes cgminer specific settings and options.

For general CGMiner information refer to doc/README inside each version (scrypt or sha256).

## Hexminer : ##

This code is forked from original cgminer v4.6.1 and official patches for hexminer were added.

Extranonce patch from nicehash were included too.

![](https://github.com/wareck/cgminer-hexminer/blob/master/patches_and_diy/images/hexminer.jpg)


How to build Scrypt version:

	sudo apt-get update
	sudo apt-get install build-essential autoconf automake libtool pkg-config libcurl4-openssl-dev libudev-dev \
	libjansson-dev libncurses5-dev libudev-dev libjansson-dev
	cd hexminer_scrypt
	./autogen.sh
	./configure --enable-scrypt --enable-hexminer
	make
	make install


How to build SHA256 version with all hexminer:

	sudo apt-get update
	sudo apt-get install build-essential autoconf automake libtool pkg-config libcurl4-openssl-dev libudev-dev \
	libjansson-dev libncurses5-dev libudev-dev libjansson-dev
	cd hexminer_sha256
	./autogen.sh
	./configure --enable-hexminera --enable-hexminerb --enable-hexmineru --enable-hexminerc --enable-hexminer8 \
	--enable-hexminerm --enable-hexminerr --enable-hexminerbe200 --enable-hexminer3
	make
	make install

How to build SHA256 version with specific hexminer:

	sudo apt-get update
	sudo apt-get install build-essential autoconf automake libtool pkg-config libcurl4-openssl-dev libudev-dev \
	libjansson-dev libncurses5-dev libudev-dev libjansson-dev
	cd hexminer_sha256
	./autogen.sh
	./configure --help
	

	wait configure finish and you can read this :
	
	--enable-hexminera      Compile support for hexminera (default disabled)
	--enable-hexminerb      Compile support for hexminerb (default disabled)
	--enable-hexmineru      Compile support for hexmineru (default disabled)
	--enable-hexminerc      Compile support for hexminerc (default disabled)
	--enable-hexminer8      Compile support for hexminer8 (default disabled)
	--enable-hexminerm      Compile support for hexminerm (default disabled)
	--enable-hexminerr      Compile support for hexminerr (default disabled)
	--enable-hexminerbe200  Compile support for hexminerbe200 (default disabled)
	--enable-hexminer3      Compile support for hexminer3 (default disabled)

	choose your miner in this list , then for example :

	./configure --enable-hexminer8
	make


### Option Summary : 

For each kind of hexminer you can have some options...

For list them: 

	./cgminer --help

```
--hexminera-options <arg> Set HEXMinerA options asic_count:freq
--hexminera-voltage <arg> Set HEXMinerA core voltage, in millivolts
--set_default_to_a  Handle USB detect errors as hexA
--hexminerb-options <arg> Set HEXMinerB options asic_count:freq
--hexminerb-voltage <arg> Set HEXMinerB core voltage, in millivolts
--set_default_to_b  Handle USB detect errors as hexB
--hexminerc-options <arg> Set HEXMinerC options asic_count:freq
--hexminerc-voltage <arg> Set HEXMinerC core voltage, in millivolts
--set_default_to_c  Handle USB detect errors as hexC
--hexmineru-frequency <arg> Set HEXMinerU frequency
--hexminer8-options <arg> Set HEXMiner8 options asic_count:freq
--hexminer8-voltage <arg> Set HEXMiner8 core voltage, in millivolts
--set_default_to_8  Handle USB detect errors as hex8
--hexminer8-chip-mask <arg> Set HEXMiner8 eneable or disable chips
--hexminer8-set-diff-to-one <arg> Set HEXMiner8 ASIC difficulty to one
--hexminerm-options <arg> Set HEXMinerM options asic_count:freq
--hexminerm-voltage <arg> Set HEXMinerM core voltage, in millivolts
--hexminerm-pic-roll <arg> Set HEXMinerM pic work roll
--set_default_to_m  Handle USB detect errors as hexM
--hexminerm-chip-mask <arg> Set HEXMinerM eneable or disable chips
--hexminerm-hw-err-res <arg> Set HEXMinerM reset chip due to N consecutive HW errors
--hexminerm-nonce-timeout-secs <arg> Set HEXMinerM reset chip due to inactivity in seconds
--hexminerm-reset-below-threshold <arg> Set HEXMinerM reset board due to low performance
--hexminerm-reset-wait <arg> Set HEXMinerM wait for 1m speed stats to settle
--hexminerr-options <arg> Set HEXMinerR options asic_count:freq
--hexminerr-voltage <arg> Set HEXMinerR core voltage, in millivolts
--hexminerr-pic-roll <arg> Set HEXMinerR pic work roll
--set_default_to_r  Handle USB detect errors as hexR
--hexminerr-chip-mask <arg> Set HEXMinerR eneable or disable chips
--hexminerr-asic-diff <arg> Set HEXMinerR ASIC difficulty
--hexminerbe200-options <arg> Set HEXMinerBE200 options asic_count:freq
--hexminerbe200-voltage <arg> Set HEXMinerBE200 core voltage, in millivolts
--hexminerbe200-pic-roll <arg> Set HEXMinerBE200 pic work roll
--hexminerbe200-asic-diff <arg> Set HEXMinerBE200 ASIC difficulty
--hexminerbe200-chip-mask <arg> Set HEXMinerBE200 eneable or disable chips
--hexminerbe200-skip-hw-res <arg> Enable or disable HEXMinerBE200 automatic reset
--hexminerbe200-hw-err-res <arg> Set HEXMinerBE200 reset chip due to N consecutive HW errors
--hexminerbe200-nonce-timeout-secs <arg> Set HEXMinerBE200 reset chip due to inactivity in seconds
--set_default_to_be200 Handle USB detect errors as hexE
--hexminer3-options <arg> Set HEXMiner3 options asic_count:freq
--hexminer3-voltage <arg> Set HEXMiner3 core voltage, in millivolts
--set_default_to_3  Handle USB detect errors as hex3
--hexminer3-chip-mask <arg> Set HEXMiner3 eneable or disable chips
```

Use this following example for base :
	
	./cgminer -o stratum+tcp://us-east.stratum.slushpool.com:3333 -u myname.worker -p x --hexminer8-set-diff-to-one 0 \
	--hexminer8-chip-mask 255 --hexminer8-voltage 1000 --hexminer8-options 8:260

![](https://github.com/wareck/cgminer-hexminer/blob/master/patches_and_diy/images/hexminer2.jpg)

