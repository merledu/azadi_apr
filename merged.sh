#!/bin/bash
git status; cd yosys_flow/orig-rtl/includes
ls *.v *.sv *.vh *.svh > flist
cd .. ;   ls *.v *.sv >> flist

while read file_name; do
	cat $file_name >>  merged;
	#echo $file_name;
done < flist

mv merged ../merged-rtl/azadi_soc_top.sv
# sv2v conversion
#sv2v azadi_soc_top.sv >  azadi_soc_top_conv.v

rm flist

echo "!!!!!!! END  !!!!!!!"
