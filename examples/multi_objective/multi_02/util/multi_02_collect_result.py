import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

infile = open(sys.argv[1], 'r') # *.txt
mode = sys.argv[2] # multi / single
order = sys.argv[3] # alternate / sequential

result_count_0 = np.zeros(shape=(2,4), dtype=int) # result_count_0[i] = [count of 00, 01, 10, 11 in segment 0] sampled from sampler i
result_count_1 = np.zeros(shape=(2,4), dtype=int) # result_count_1[i] = [count of 00, 01, 10, 11 in segment 1] sampled from sampler i
curr_source = 0
lines = infile.readlines()
infile.close()

for i in range(len(lines)):
    if mode == 'multi':
        if 'Rho' in lines[i]:
            line = lines[i].strip()
            seg1 = line[line.find('[[')+2:line.find(']')].split(' ')
            val1 = []
            for s in seg1:
                if s != '':
                    val1.append(float(s) < 0)
            assert len(val1) == 4, 'Invalid length of rho'
            result_count_0[curr_source][val1[0]*2 + val1[1]*1] += 1

            line = lines[i+1].strip()
            seg2 = line[line.find('[')+1:line.find(']]')].split(' ')
            val2 = []
            for s in seg2:
                if s != '':
                    val2.append(float(s) < 0)
            assert len(val2) == 4, 'Invalid length of rho'
            result_count_1[curr_source][val2[2]*2 + val2[3]] += 1

            if order == 'alternate':
                curr_source = 1 - curr_source
    else:
        if 'Actual rho' in lines[i]:
            line = lines[i].strip()
            seg1 = line[line.find('[')+1:line.find(']')].split(' ')
            val1 = []
            for s in seg1:
                if s != '':
                    val1.append(float(s) < 0)
            assert len(val1) == 4, 'Invalid length of rho'
            result_count_0[curr_source][val1[0]*2 + val1[1]*1] += 1

            seg2 = line[line.find('] [')+3:-1].split(' ')
            val2 = []
            for s in seg2:
                if s != '':
                    val2.append(float(s) < 0)
            assert len(val2) == 4, 'Invalid length of rho'
            result_count_1[curr_source][val2[2]*2 + val2[3]] += 1
            
rows = ['from sampler 0', 'from sampler 1']
cols = ['(r0, r1) = 00', '(r0, r1) = 01', '(r0, r1) = 10', '(r0, r1) = 11']
df = pd.DataFrame(result_count_0, columns=cols, index=rows)
print('Falsification result in segment 0:\n', df, '\n')
cols = ['(r2, r3) = 00', '(r2, r3) = 01', '(r2, r3) = 10', '(r2, r3) = 11']
df = pd.DataFrame(result_count_1, columns=cols, index=rows)
print('Falsification result in segment 1:\n', df)
