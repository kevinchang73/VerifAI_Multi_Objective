import numpy as np
import networkx as nx
from itertools import product
from verifai.samplers.domain_sampler import BoxSampler, DiscreteBoxSampler, \
    DomainSampler, SplitSampler
from verifai.samplers.random_sampler import RandomSampler
from verifai.samplers.cross_entropy import DiscreteCrossEntropySampler
from verifai.samplers.multi_objective import MultiObjectiveSampler
from verifai.rulebook import rulebook

class DynamicExtendedMultiArmedBanditSampler(DomainSampler):
    sampler_idx = 0
    
    def __init__(self, domain, demab_params):
        print('(dynamic_emab.py) Initializing!!!')
        print('(dynamic_emab.py) demab_params =', demab_params)
        super().__init__(domain)
        self.alpha = demab_params.alpha
        self.thres = demab_params.thres
        self.cont_buckets = demab_params.cont.buckets
        self.cont_dist = demab_params.cont.dist
        self.disc_dist = demab_params.disc.dist
        self.cont_ce = lambda domain: ContinuousDynamicEMABSampler(domain=domain,
                                                     buckets=self.cont_buckets,
                                                     dist=self.cont_dist,
                                                     alpha=self.alpha,
                                                     thres=self.thres)
        self.disc_ce = lambda domain: DiscreteDynamicEMABSampler(domain=domain,
                                                   dist=self.disc_dist,
                                                   alpha=self.alpha,
                                                   thres=self.thres)
        partition = (
            (lambda d: d.standardizedDimension > 0, self.cont_ce),
            (lambda d: d.standardizedIntervals, self.disc_ce)
        )
        self.split_samplers = {}
        for id, priority_graph in rulebook.priority_graphs.items():
            self.split_samplers[id] = SplitSampler.fromPartition(domain,
                                                                partition,
                                                                RandomSampler)
            for subsampler in self.split_samplers[id].samplers:
                if isinstance(subsampler, ContinuousDynamicEMABSampler):
                    print('(dynamic_emab.py) Set priority graph', id)
                    subsampler.set_graph(priority_graph)
                    subsampler.compute_error_weight()
                elif isinstance(subsampler, DiscreteDynamicEMABSampler):
                    assert True
                else:
                    assert isinstance(subsampler, RandomSampler)
            node_ids = list(nx.dfs_preorder_nodes(priority_graph))
            if not sorted(node_ids) == list(range(len(node_ids))):
                raise ValueError('Node IDs should be in order and start from 0')
        if not sorted(list(self.split_samplers.keys())) == list(range(len(rulebook.priority_graphs))):
            raise ValueError('Priority graph IDs should be in order and start from 0')
        self.num_segs = len(self.split_samplers)
        print('(dynamic_emab.py) num_segs =', self.num_segs)

    def getSample(self):
        # Sample from each segment in a round-robin fashion
        idx = self.sampler_idx % self.num_segs
        return self.split_samplers[idx].getSample()

    def update(self, sample, info, rhos):
        # Update each sampler based on the corresponding segment
        try:
            iter(rhos)
        except:
            for i in range(len(self.split_samplers)):
                self.split_samplers[i].update(sample, info, rhos)
            return
        print('(dynamic_emab.py) Getting feedback from segment', self.sampler_idx % self.num_segs)
        for i in range(len(rhos)):
            self.split_samplers[i].update(sample, info, rhos[i])
        self.sampler_idx += 1

class ContinuousDynamicEMABSampler(BoxSampler, MultiObjectiveSampler):
    verbosity = 2

    def __init__(self, domain, alpha, thres,
                 buckets=10, dist=None, restart_every=100):
        super().__init__(domain)
        if isinstance(buckets, int):
            buckets = np.ones(self.dimension) * buckets
        elif len(buckets) > 1:
            assert len(buckets) == self.dimension
        else:
            buckets = np.ones(self.dimension) * buckets[0]
        if dist is not None:
            assert (len(dist) == len(buckets))
        if dist is None:
            dist = np.array([np.ones(int(b))/b for b in buckets])
        self.buckets = buckets # 1*d, each element specifies the number of buckets in that dimension
        self.dist = dist # N*d, ???
        self.alpha = alpha
        self.thres = thres
        self.current_sample = None
        self.counts = np.array([np.ones(int(b)) for b in buckets]) # N*d, T (visit times)
        self.errors = np.array([np.zeros(int(b)) for b in buckets]) # N*d, total times resulting in maximal counterexample
        self.t = 1 # time, used in Q
        self.counterexamples = dict()
        self.is_multi = True #False
        self.invalid = np.array([np.zeros(int(b)) for b in buckets]) # N*d, ???
        self.monitor = None
        self.rho_values = []
        self.restart_every = restart_every

    def getVector(self):
        return self.generateSample()
    
    def generateSample(self):
        proportions = self.errors / self.counts
        Q = proportions + np.sqrt(2 / self.counts * np.log(self.t))
        # choose the bucket with the highest "goodness" value, breaking ties randomly.
        bucket_samples = np.array([np.random.choice(np.flatnonzero(np.isclose(Q[i], Q[i].max())))
            for i in range(len(self.buckets))])
        self.current_sample = bucket_samples
        ret = tuple(np.random.uniform(bs, bs+1.)/b for b, bs
              in zip(self.buckets, bucket_samples)) # uniform randomly sample from the range of the bucket
        return ret, bucket_samples
    
    def updateVector(self, vector, info, rho):
        assert rho is not None
        # "random restarts" to generate a new topological sort of the priority graph
        # every restart_every samples.
        if self.is_multi:
            if self.monitor is not None and self.monitor.linearize and self.t % self.restart_every == 0:
                self.monitor._linearize()
            self.update_dist_from_multi(vector, info, rho)
            return
        self.t += 1
        for i, b in enumerate(info):
            self.counts[i][b] += 1.
            if rho < self.thres:
                self.errors[i][b] += 1.

    def is_better_counterexample(self, ce1, ce2):
        if ce2 is None:
            return True
        return self._compute_error_value(ce1) > self._compute_error_value(ce2)

    def _get_total_counterexamples(self):
        return sum(self.counterexamples.values())
    
    def _update_counterexample(self, ce, to_delete=False): # update counterexamples, may or may not delete non-maximal counterexamples
        if ce in self.counterexamples:
            return True
        if to_delete:
            to_remove = set()
            if len(self.counterexamples) > 0:
                for other_ce in self.counterexamples:
                    if self.is_better_counterexample(other_ce, ce):
                        return False
            for other_ce in self.counterexamples:
                if self.is_better_counterexample(ce, other_ce):
                    to_remove.add(other_ce)
            for other_ce in to_remove:
                del self.counterexamples[other_ce]
        self.counterexamples[ce] = np.array([np.zeros(int(b)) for b in self.buckets])
        return True
    
    def update_dist_from_multi(self, sample, info, rho):
        try:
            iter(rho)
        except:
            for i, b in enumerate(info):
                self.invalid[i][b] += 1.
            return
        if len(rho) != self.num_properties:
            for i, b in enumerate(info):
                self.invalid[i][b] += 1.
            return
        
        counter_ex = tuple(rho[node] < self.thres[node] for node in sorted(self.priority_graph.nodes))
        error_value = self._compute_error_value(counter_ex)
        self._update_counterexample(counter_ex)
        for i, b in enumerate(info):
            self.counts[i][b] += 7.
            self.counterexamples[counter_ex][i][b] += error_value
        self.errors = self._get_total_counterexamples()
        self.t += 1
        if self.verbosity >= 2:
            print('counterexamples =', self.counterexamples)
        if self.verbosity >= 1:
            for ce in self.counterexamples:
                if self._compute_error_value(ce) > 0:
                    print('counterexamples =', ce, ', times =', int(np.sum(self.counterexamples[ce], axis = 1)[0]/self._compute_error_value(ce)))
        if self.verbosity >= 2:
            proportions = self.errors / self.counts
            print('self.errors[0] =', self.errors[0])
            print('self.counts[0] =', self.counts[0])
            Q = proportions + np.sqrt(2 / self.counts * np.log(self.t))
            print('Q[0] =', Q[0], '\nfirst_term[0] =', proportions[0], '\nsecond_term[0] =', np.sqrt(2 / self.counts * np.log(self.t))[0], '\nratio[0] =', proportions[0]/(proportions+np.sqrt(2 / self.counts * np.log(self.t)))[0])

    def _compute_error_value(self, counter_ex):
        error_value = 0
        for i in range(len(counter_ex)):
            error_value += 2**(self.error_weight[i]) * counter_ex[i]
        return error_value
    
    def compute_error_weight(self):
        level = {}
        for node in nx.topological_sort(self.priority_graph):
            if self.priority_graph.in_degree(node) == 0:
                level[node] = 0
            else:
                level[node] = max([level[p] for p in self.priority_graph.predecessors(node)]) + 1
        
        ranking_map = {}
        ranking_count = {}
        for rank in sorted(level.values()):
            if rank not in ranking_count:
                ranking_count[rank] = 1
            else:
                ranking_count[rank] += 1
        count = 0
        for key, value in reversed(ranking_count.items()):
            ranking_map[key] = count
            count += value
        
        self.error_weight = {} #node_id -> weight
        for node in level:
            if self.priority_graph.nodes[node]['active']:
                self.error_weight[node] = ranking_map[level[node]]
            else:
                self.error_weight[node] = -1
        for key, value in sorted(self.error_weight.items()):
            if self.verbosity >= 2:
                print(f"Node {key}: {value}")

class DiscreteDynamicEMABSampler(DiscreteCrossEntropySampler):
    pass
