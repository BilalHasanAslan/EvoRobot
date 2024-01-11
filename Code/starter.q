#!/bin/bash
#PBS -l select=1:ncpus=24:mpiprocs=24:mem=30gb
#PBS -P CSCI1142
#PBS -q smp
#PBS -l walltime=96:00:00
#PBS -o /mnt/lustre/users/baslan/2/data/test1.out
#PBS -e /mnt/lustre/users/baslan/2/data/test1.err
#PBS -m abe
#PBS -M aslbil001@myuct.ac.za
ulimit -s unlimited

cd /mnt/lustre/users/baslan/cleanedevo/5start
nproc = cat $PBS_NODEFILE | wc -l
echo nproc is $nproc
cat $PBS_NODEFILE

module purge
eval "$(conda shell.bash hook)"
conda activate /home/baslan/myenv
python3 mle.py 'sim3600rampice' 0.05 0.05 0.05 0.05 1000 4000 1 1
