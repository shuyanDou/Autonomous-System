# Copyright (C) 2024 OpenDLV

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y \
    build-essential \
    cmake \
    software-properties-common \
    libopencv-dev

RUN add-apt-repository 'ppa:chrberger/libcluon' && \
    apt-get update && \
    apt-get install libcluon

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y && \
    apt-get install -y --no-install-recommends \
        python3-protobuf \
        python3-sysv-ipc \        
        python3-numpy \
        python3-opencv \
        protobuf-compiler
RUN apt-get clean

ADD . /opt/sources
WORKDIR /opt/sources
RUN make

ENTRYPOINT ["/opt/sources/opendlv-perception-helloworld.py"]
