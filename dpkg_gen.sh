cp ./flightctrl_proxy  /mnt/workspace/Company/IOT/Dpkg_Make/UFA5/UFA5_8x74/usr/bin/
cp ./nfz.info  /mnt/workspace/Company/IOT/Dpkg_Make/UFA5/UFA5_8x74/etc/fctrl_proxy/
cd /mnt/workspace/Company/IOT/Dpkg_Make/UFA5
pwd
rm -rf *.deb
dpkg -b /mnt/workspace/Company/IOT/Dpkg_Make/UFA5/UFA5_8x74/  ufa5_flightctrl_proxy_1.2.1.8x74.deb
