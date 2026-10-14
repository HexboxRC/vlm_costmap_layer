"""Summarize raw request records without conflating clocks or hiding failures."""
from collections import Counter
import math

FIELDS = ('attempt_elapsed_s','laptop_roundtrip_s','pi_provider_s','pi_total_s',
          'capture_to_grid_s','sim_age_at_publish_s','sim_age_at_apply_s')


def elapsed_seconds(start_ns, end_ns):
    """Same-clock duration only; negative values are missing, never fabricated zero."""
    if start_ns is None or end_ns is None or end_ns < start_ns:
        return None
    return (end_ns-start_ns)*1e-9


def quantile(values, probability):
    """Linear interpolation between ordered samples (including for small n)."""
    values=sorted(values)
    if not values:return None
    position=(len(values)-1)*probability
    low=math.floor(position); high=math.ceil(position)
    return values[low]+(values[high]-values[low])*(position-low)


def summarize(records):
    records=list(records)
    result={'attempted':len(records),'status_counts':dict(Counter(r['status'] for r in records)),
            'censored_roundtrips':sum(r['status'] in ('timeout','transport_error') for r in records),
            'roundtrip_percentiles_are_conditional_on_response':True,'durations':{}}
    for key in FIELDS:
        available=[r[key] for r in records if r.get(key) is not None and
                   math.isfinite(r[key]) and r[key]>=0 and
                   not (key=='laptop_roundtrip_s' and r['status'] in ('timeout','transport_error'))]
        result['durations'][key]={'n':len(available),'missing_or_invalid':len(records)-len(available),
            'median':quantile(available,.5),'p90':quantile(available,.9),
            'p95':quantile(available,.95),'p99':quantile(available,.99),
            'max':max(available) if available else None,
            'sorted_samples':sorted(available)}
    return result
