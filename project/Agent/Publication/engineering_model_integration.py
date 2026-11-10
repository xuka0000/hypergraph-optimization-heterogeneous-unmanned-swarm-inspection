"""Identity and readiness checks for the integrated definition, not a solver."""
from pathlib import Path
import json


def readiness(root, legacy_diagnostic=False):
    root=Path(root)
    contract=json.loads((root/'Agent/Publication/engineering_inspection_contract.json').read_text('utf-8-sig'))
    revision=contract.get('integration_revision')
    if not revision:
        return dict(allowed=True,scope='existing pre-integration definition',revision=contract.get('revision'))
    if legacy_diagnostic:
        return dict(allowed=True,scope='legacy_deterministic_day_ahead_diagnostic_only',
                    revision=contract['revision'],complete_integrated_model=False)
    # Closed-loop evidence must be obtained by running the qualified new
    # implementation; requiring that evidence to launch it would be circular.
    required=('complete_numeric_instance_bound','runtime_parity_verified')
    missing=[name for name in required if revision.get(name) is not True]
    return dict(allowed=not missing,scope='current_integrated_model',revision=contract['revision'],
                complete_integrated_model=not missing,
                closed_loop_verified=revision.get('closed_loop_verified') is True,missing=missing,
                missing_parameter_groups=revision.get('missing_parameter_groups',[]),
                reason='The retained regional executor is the earlier deterministic development model; do not label it as the refined integrated model.')


def compatible_power_model(aircraft_platform, model_platform):
    """No numerical scaling or silent substitution can create a new calibration."""
    return bool(aircraft_platform and model_platform and aircraft_platform==model_platform)
