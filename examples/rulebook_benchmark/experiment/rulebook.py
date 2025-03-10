class Rulebook:
    def __init__(self, rules):
        self.rules = rules
    
    def apply(self, realization, **kwargs):
        violations = []
        violation_histories = []
        for rule in self.rules:
            violation, violation_history = rule(realization, **kwargs)
            violations.append(violation)
            violation_histories.append(violation_history)
            
        return violations, violation_histories
    
    
class Rule:
    def __init__(self, rule, name, description):
        self.rule = rule
        self.name = name
        self.description = description
    
    def __call__(self, realization, **kwargs):
        return self.rule(realization, **kwargs)
    
    