FROM ubuntu:latest

RUN apt-get update && \
	apt-get install -y build-essential git cmake autoconf libtool pkg-config

WORKDIR /src

COPY ./ ./

WORKDIR /build

RUN cmake /src && make

CMD ["/build/integration_tests/integration_tests"]
