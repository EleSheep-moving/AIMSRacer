from aims_mpcc.benchmark import summarize


def test_tail_and_failure_counts_include_every_request():
    rows = [dict(success=True, full_request_s=.01),
            dict(success=False, full_request_s=.2),
            dict(success=True, full_request_s=.03)]
    result = summarize(rows, budget_s=.05)
    assert result['requests'] == 3
    assert result['failures'] == 1
    assert result['overruns'] == 1
    assert result['max_consecutive_overruns'] == 1
    assert result['full_request_p95_s'] > .18


def test_consecutive_overruns_and_empty_run_are_explicit():
    assert summarize([], .05)['full_request_p95_s'] is None
    rows = [dict(success=False, full_request_s=.1) for _ in range(3)]
    assert summarize(rows, .05)['max_consecutive_overruns'] == 3
