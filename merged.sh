#!/bin/bash
git status;

# for includes and packages
cd ./yosys_flow/orig-rtl/includes
ls *.v *.sv *.vh *.svh > flist1

while read file_name; do
	cat $file_name >> ../merged;
	echo $file_name;
done < flist1
rm flist1

# for rtl models
cd ../
ls *.v *.sv > flist2

while read file_name; do
        cat $file_name >>  merged;
        echo $file_name;
done < flist2

mv merged ../merged-rtl/azadi_soc_top.sv
# sv2v conversion
#sv2v azadi_soc_top.sv >  azadi_soc_top_conv.v

rm flist2

echo "!!!!!!! END  !!!!!!!"
