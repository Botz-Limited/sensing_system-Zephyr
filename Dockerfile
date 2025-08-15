FROM golang:1.20.5 AS go-build

ENV GOPATH="/usr/local/go"
ENV PATH="${PATH}:$GOPATH/bin"
ENV BOARD_ROOT=/home/ee/sensing_fw
ENV APPLICATION_ROOT_PATH=/home/ee/sensing_fw

#Ideal for scratch docker image deployments - as no host OS needs to be bundled.
ENV CGO_ENABLED=0

# Install packages
RUN go install github.com/apache/mynewt-mcumgr-cli/mcumgr@latest \
	&& cp -r ${GOPATH}/bin /usr/local

FROM ubuntu:24.04

COPY --from=go-build /usr/local /usr/local

ARG NCS_VERSION=3.1.0
ARG NRFTOOLS_VERSION=10.24.2
ARG ZSDK_VERSION=0.17.0
ARG GCC_VERSION=12
ARG CMAKE_VERSION=4.0.2
ARG CPPCHECK_VERSION=2.17.1
ARG LLVM_VERSION=18
ARG WGET_ARGS="-q --show-progress --progress=bar:force:noscroll --no-check-certificate"

ARG UID=1000
ARG GID=1000

ENV BOARD_ROOT=/home/ee/sensing_fw
ENV APPLICATION_ROOT_PATH=/home/ee/sensing_fw

# Set default shell during Docker image build to bash
SHELL ["/bin/bash", "-c"]

# Set non-interactive frontend for apt-get to skip any user confirmations
ENV DEBIAN_FRONTEND=noninteractive
ENV APPLICATION_ROOT_PATH=/home/ee/sensing_fw
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV PKG_CONFIG_PATH=/usr/lib/i386-linux-gnu/pkgconfig
ENV OVMF_FD_PATH=/usr/share/ovmf/OVMF.fd
ENV ZEPHYR_BASE=/home/ee/ncs/zephyr
ENV CMAKE_PREFIX_PATH=/opt/toolchains
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/toolchains/zephyr-sdk-zephyr-sdk-${ZSDK_VERSION}
ENV NANOPB_PATH=/home/ee/ncs/modules/lib/nanopb
ENV PATH="${PATH}:${NANOPB_PATH}/generator" 
ENV PYTHONPATH="${NANOPB_PATH}/generator/proto"
ENV MERMAID_FILTER_WIDTH=4000
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

# Install base packages
RUN apt-get -y update && \
	apt-get -y upgrade && \
	apt-get install --no-install-recommends -y \
	build-essential \
	ca-certificates \
	ccache \
	chromium-browser \
	clang-format \
	cppcheck \
	curl \
	device-tree-compiler \
	dfu-util \
	file \
	fonts-dejavu \
	gcovr \
	gdb \
	git \
	git-core \
	gn \
	gperf \
	gpg-agent \
	lcov \
	less \
	libatk1.0-0 \
	libatk-bridge2.0-0 \
	libc6 \
	libfontconfig1 \
	libfreetype6 \
	libgbm-dev \
	libgdk-pixbuf2.0-0 \
	libgtk-3-0 \
	libncurses6 \
	libnss3-dev \
	libpcre3-dev \
	librsvg2-bin \
	libsdl2-dev \
	libxss-dev \
	lmodern \
	locales \
	lsb-release \
	make \
	ninja-build \
	openssh-client \
	pandoc \
	pkg-config \
	protobuf-compiler \
	python3-dev \
	python3-requests \
	python3-venv \
	python3-pip \
	python3-ply \
	python3-setuptools \
	python-is-python3 \
	qemu-system \
	ruby-full \
	software-properties-common \
	sudo \
	texlive-fonts-recommended \
	texlive-latex-extra \
	texlive-latex-recommended \
	texlive-pictures \
	texlive-xetex \
	udev \
	unzip \
	valgrind \
	wget \
	doxygen \
	xz-utils 

# Install multi-lib gcc (x86 only) + fix for missing asm/errno.h:
RUN if [ "${HOSTTYPE}" = "x86_64" ]; then \
	apt-get install --no-install-recommends -y \
	gcc-${GCC_VERSION}-multilib \
	g++-${GCC_VERSION}-multilib \
	linux-libc-dev-i386-cross && \
	ln -s /usr/include/x86_64-linux-gnu/asm /usr/include/asm \
	; fi

# Initialise system locale
RUN locale-gen en_GB.UTF-8
ENV LANG=en_GB.UTF-8
ENV LANGUAGE=en_GB:en
ENV LC_ALL=en_GB.UTF-8


RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-${GCC_VERSION} 10 && \
	update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${GCC_VERSION} 10

# Install CMake
RUN wget ${WGET_ARGS} https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-${HOSTTYPE}.sh && \
	chmod +x cmake-${CMAKE_VERSION}-Linux-${HOSTTYPE}.sh && \
	./cmake-${CMAKE_VERSION}-Linux-${HOSTTYPE}.sh --skip-license --prefix=/usr/local && \
	rm -f ./cmake-${CMAKE_VERSION}-Linux-${HOSTTYPE}.sh

# Install LLVM and Clang
RUN wget ${WGET_ARGS} https://apt.llvm.org/llvm.sh && \
	chmod +x llvm.sh && \
	sudo ./llvm.sh ${LLVM_VERSION} all && \
	update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 10 && \
	update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 10 && \
	update-alternatives --install /usr/bin/clang-tidy clang-clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 10


# Gets an updated, known version of cppcheck
RUN wget ${WGET_ARGS} https://github.com/danmar/cppcheck/archive/refs/tags/${CPPCHECK_VERSION}.zip -O cppcheck-${CPPCHECK_VERSION}.zip && \
	unzip cppcheck-${CPPCHECK_VERSION}.zip && \
	mkdir cppcheck-${CPPCHECK_VERSION}/build && \
	cd cppcheck-${CPPCHECK_VERSION}/build && \
	cmake .. -DUSE_MATCHCOMPILER=ON -DHAVE_RULES=ON -DCMAKE_BUILD_TYPE=Release && \
	cmake --build . && \
	make install && \
	cd ../.. && \
	rm cppcheck-${CPPCHECK_VERSION}.zip && \
	rm -R cppcheck-${CPPCHECK_VERSION}

# Install Zephyr SDK
RUN mkdir -p /opt/toolchains
WORKDIR /opt/toolchains

RUN	wget ${WGET_ARGS} https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz &&\
	tar xf zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz &&\
	zephyr-sdk-${ZSDK_VERSION}/setup.sh -t all -h -c && \
 	wget ${WGET_ARGS} https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/toolchain_linux-${HOSTTYPE}_arm-zephyr-eabi.tar.xz &&\
	tar xf toolchain_linux-${HOSTTYPE}_arm-zephyr-eabi.tar.xz -C zephyr-sdk-${ZSDK_VERSION}/ &&\
	rm zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz &&\
	rm toolchain_linux-${HOSTTYPE}_arm-zephyr-eabi.tar.xz

WORKDIR /opt

# Install nRF programming tools
RUN wget ${WGET_ARGS} https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/$(echo ${NRFTOOLS_VERSION} | sed -r 's/[.]+/-/g')/nrf-command-line-tools_${NRFTOOLS_VERSION}_amd64.deb && \
    sudo apt-get install -y ./nrf-command-line-tools_${NRFTOOLS_VERSION}_amd64.deb && \
    sudo rm ./nrf-command-line-tools_${NRFTOOLS_VERSION}_amd64.deb

WORKDIR /

# Install JLink provided in nRF programming tools
RUN mkdir jlink_unpacked && \
    dpkg-deb -R /opt/nrf-command-line-tools/share/JLink*.deb jlink_unpacked && \
    rm jlink_unpacked/DEBIAN/postinst && \
    dpkg-deb -b jlink_unpacked jlink_fixed.deb && \
    apt-get install -y ./jlink_fixed.deb && \
	rm -R jlink_unpacked

# Install nrfutil
RUN wget ${WGET_ARGS} https://github.com/NordicSemiconductor/pc-nrfutil/releases/download/v6.1.7/nrfutil-linux && \
	mv nrfutil-linux /bin/nrfutil && \
	chmod +x /bin/nrfutil

RUN mkdir -p ${APPLICATION_ROOT_PATH}

# Final Update Check
RUN apt-get -y update && \
	apt-get -y upgrade

# Clean up stale packages
RUN apt-get clean -y && \
	apt-get autoremove --purge -y && \
	rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_22.x -o nodesource_setup.sh && \
	bash nodesource_setup.sh && \
	apt-get install -y nodejs && \
	npm i -g mermaid-filter

# Install nRF Connect SDK
WORKDIR /home/ee

# Create and activate venv (but must prefix each pip command with the venv path)
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# 3. Install Python packages via pip (now uses venv automatically)
RUN pip install --upgrade pip && \
    pip install west


RUN west init -m https://github.com/nrfconnect/sdk-nrf --mr v${NCS_VERSION} ncs

WORKDIR /home/ee/ncs

RUN west update && \
	west zephyr-export

#RUN west sdk install

RUN pip install -r zephyr/scripts/requirements.txt && \
    pip install -r nrf/scripts/requirements.txt && \
    pip install -r bootloader/mcuboot/scripts/requirements.txt && \
    pip install \	protobuf==5.29.4 \
	grpcio-tools==1.71.0 \
	sh \
	awscli \
	PyGithub \
	junitparser \
	pylint \
	statistics \
	numpy \
	imgtool \
	GitPython \
	pyelftools \
	pykwalify \
	psutil \
	pyserial \
	anytree \
	tabulate

WORKDIR /

# Install oh-my-bash
RUN bash -c "$(wget https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh -O -)"

RUN sudo -E -- bash -c ' \
	/opt/toolchains/zephyr-sdk-${ZSDK_VERSION}/setup.sh -t arm-zephyr-eabi -h -c && \
	ln -s /opt/toolchains/zephyr-sdk-${ZSDK_VERSION}/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb /opt/toolchains/zephyr-sdk-${ZSDK_VERSION}/arm-zephyr-eabi/bin/arm-none-eabi-gdb \
	'
# Launch bash shell by default
CMD ["/bin/bash"]
