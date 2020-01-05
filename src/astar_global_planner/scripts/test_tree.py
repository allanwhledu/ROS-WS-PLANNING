from collections import defaultdict
import json


def tree(): return defaultdict(tree)

users = tree()
users['harold']['username'] = 'hrldcpr'
users['handler']['username'] = 'matthandlersux'

print(json.dumps(users))