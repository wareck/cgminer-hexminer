# How-to do it by yourself :
==============

(compile and patch your own software)

### Prerequise:

	sudo apt-get update
	sudo apt-get install build-essential autoconf automake libtool pkg-config libcurl3-dev libudev-dev -y

### Download Sources:

(download cgminer v4.6.1)
	
	git clone https://github.com/ckolivas/cgminer.git
	cd cgminer
	git checkout 9afd0a216a0f95adb650e4818f24af1a61ad837d


### Download patches:

Sha256 Hexminer patch :

	wget https://raw.githubusercontent.com/wareck/cgminer-hexminer/master/patches_and_diy/rev_9afd0a216a0f95adb650e4818f24af1a61ad837d.patch

Scrypt Hexminer patch :

	wget https://raw.githubusercontent.com/wareck/cgminer-hexminer/master/patches_and_diy/srev_9afd0a216a0f95adb650e4818f24af1a61ad837d.patch

Extranonce optimisation (for nicehash and similar sites):

	wget https://raw.githubusercontent.com/wareck/cgminer-hexminer/master/patches_and_diy/extranonce.patch

### Patching :

If you want to use sha256 Hexminer :

	patch -p1 <rev_9afd0a216a0f95adb650e4818f24af1a61ad837d.patch

If you want to use Scrypt Hexminer :

	patch -p1 <srev_9afd0a216a0f95adb650e4818f24af1a61ad837d.patch

If you want add extranonce options (nicehash optminisation):

	patch -p1 <extranonce.patch

### Compilation:


Sha256 Hexminer:

	./autogen.sh --enable-hexminera --enable-hexminerb  --enable-hexmineru --enable-hexminerc  --enable-hexminer8 /
	--enable-hexminerm --enable-hexminerr --enable-hexminerbe200 --enable-hexminer3 

	./configure --enable-hexminera --enable-hexminerb  --enable-hexmineru --enable-hexminerc  --enable-hexminer8 /
	--enable-hexminerm --enable-hexminerr --enable-hexminerbe200 --enable-hexminer3 
	make
	make install

Scrypt Hexminer:

	./autogen.sh --enable-hexminers
	./configure --enable-hexminers
	make
	make install
