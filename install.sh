sudo apt-get update
sudo apt-get install libhiredis-dev
git clone https://github.com/sewenew/redis-plus-plus
cd redis-plus-plus
mkdir build
cd build
cmake ..
make
sudo make install
cd ..
sudo apt-get install libmemcached-dev
sudo apt install memcached
sudo apt install libmemcached-tools
mv ./memcached.conf /etc/memcached.conf
sudo systemctl start memcached
sudo chmod 777 /var/run/memcached
sudo ldconfig
./build.sh
