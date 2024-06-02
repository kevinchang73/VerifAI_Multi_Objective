iteration=300
scenario='multi_hri'
log_file="result_${scenario}_demab1.log"
result_file="result_${scenario}_demab1.txt"
csv_file="result_${scenario}_demab1"
sampler_idx=1 # 0 / 1 / -1 (-1 is for alternate)
sampler_type=demab # demab / dmab / random / dce / halton / udemab
simulator=scenic.simulators.habitat.model # scenic.simulators.carla.model / scenic.simulators.newtonian.driving_model
to_plot="False" # True / False
seed=1
rm $scenario/outputs/*traj*.txt
rm $scenario/outputs/*traj*.png
rm $scenario/outputs/$log_file
rm $scenario/outputs/$result_file
rm $scenario/outputs/$csv_file.csv
rm $scenario/outputs/$csv_file\_scatter.png

python $scenario/$scenario.py -n $iteration --headless -e $csv_file.$seed -sp $scenario/$scenario.scenic -gp $scenario/ -rp $scenario/$scenario\_spec.py -s $sampler_type --seed $seed --using-sampler $sampler_idx -m $simulator -co $scenario/outputs >> $scenario/outputs/test_error.log
#for seed in $(seq 0 2);
#do
    #if [ "$to_plot" = "True" ]; then
        #python $scenario/$scenario.py -n $iteration --headless -e $csv_file.$seed -sp $scenario/$scenario.scenic -gp $scenario/ -rp $scenario/$scenario\_spec.py -s $sampler_type --seed $seed --using-sampler $sampler_idx -m $simulator -co $scenario/outputs -o $scenario/outputs >> $scenario/outputs/$log_file
        ##for i in $(seq 0 $(($iteration-1)));
        ##do
        ##    python $scenario/util/$scenario\_plot_traj.py $scenario/outputs/$scenario\_traj_$i.txt
        ##done
    #else
        #python $scenario/$scenario.py -n $iteration --headless -e $csv_file.$seed -sp $scenario/$scenario.scenic -gp $scenario/ -rp $scenario/$scenario\_spec.py -s $sampler_type --seed $seed --using-sampler $sampler_idx -m $simulator -co $scenario/outputs >> $scenario/outputs/$log_file
    #fi
#done
#python $scenario/util/$scenario\_collect_result.py $scenario/outputs/$log_file multi $sampler_idx >> $scenario/outputs/$result_file
#python $scenario/util/$scenario\_analyze_diversity.py $scenario/outputs/ $csv_file multi >> $scenario/outputs/$result_file
