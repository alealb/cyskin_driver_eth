
ARG BASE_IMG=ubuntu:22.04
FROM $BASE_IMG

ENV DEBIAN_FRONTEND=noninteractive

# Install libskin dependencies
RUN apt-get update
# Eigen
RUN apt install -y libeigen3-dev
# Json
RUN apt install -y nlohmann-json3-dev
# Install dep for developement
RUN apt install -y build-essential cmake
RUN apt install -y python3-pip
RUN apt-get -y install clang-format
RUN pip install cpplint
RUN pip install pre-commit

# Install tmux
RUN apt-get install -y tmux

# Install openssh-client for ssh-keyscan
RUN apt-get install -y openssh-client
# Add ssh private key into container
ARG SSH_PRIVATE_KEY
RUN mkdir ~/.ssh/
RUN echo "${SSH_PRIVATE_KEY}" > ~/.ssh/id_rsa
RUN chmod 600 ~/.ssh/id_rsa
RUN ssh-keyscan github.com >> ~/.ssh/known_hosts

#  Install libskin
RUN apt-get install -y git
WORKDIR /home/
RUN git clone git@github.com:ORI-SRL/libskin.git
WORKDIR /home/libskin
RUN mkdir build
WORKDIR /home/libskin/build
RUN cmake .. -DBUILD_TESTING=OFF -DBUILD_PYTHON=OFF -DCMAKE_BUILD_TYPE=Release
RUN make -j4
RUN make install

# Install SOEM
WORKDIR /home/
RUN git clone https://github.com/OpenEtherCATsociety/SOEM.git
WORKDIR /home/SOEM
RUN git checkout v1.4.0
RUN mkdir build
WORKDIR /home/SOEM/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS="-Wno-error=stringop-truncation" ..
RUN make -j4
RUN make install

# Set the working directory
WORKDIR /home/cyskin_driver_eth

