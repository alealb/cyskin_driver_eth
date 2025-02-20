FROM cyskin_driver_eth-dev

# Set the workdir
WORKDIR /home/cyskin_driver_eth

# Copy the source code
COPY src/ /home/cyskin_driver_eth/src/
COPY CMakeLists.txt /home/cyskin_driver_eth/CMakeLists.txt
COPY include/ /home/cyskin_driver_eth/include/

# Build the project
WORKDIR /home/cyskin_driver_eth/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

# Set permissions
RUN chmod +x /home/cyskin_driver_eth/build/cyskin_driver_eth

# Clean the apt cache
RUN rm -rf /var/lib/apt/lists/*

# Set the entrypoint and parameters
ENTRYPOINT ["/home/cyskin_driver_eth/build/cyskin_driver_eth"]
CMD []