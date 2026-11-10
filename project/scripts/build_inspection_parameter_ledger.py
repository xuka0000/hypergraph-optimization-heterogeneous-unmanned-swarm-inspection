"""Index current inputs without silently filling calibration or model gaps."""
from pathlib import Path
import collections,datetime,hashlib,json
ROOT=Path(__file__).resolve().parents[1]
BOOK=ROOT/'总结/超图优化与无人集群巡检建模专著'
SCENE=ROOT/'总结/03_实验数据/真实地理巡检场景共享/scenes/rommerskirchen.json'
CONTRACT=ROOT/'Agent/Publication/engineering_inspection_contract.json'
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def bind(p):return dict(path=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def save(p,d):
 p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def walk(obj,prefix=()):
 for key,value in obj.items():
  path=(*prefix,key)
  if isinstance(value,dict):yield from walk(value,path)
  else:yield path,value
def main():
 scene=read(SCENE);contract=read(CONTRACT);rows=[]
 inherited={'camera.image_width_px':'H20','camera.image_height_px':'H20','camera.diagonal_fov_deg_at_wide':'H20',
  'camera.focal_length_mm_bounds':'H20','camera.gimbal_pitch_deg':'H20','camera.gimbal_yaw_deg':'H20',
  'energy.pair_capacity_Wh':'M300',
  'geometry.tower_height_m':'KPLC25','geometry.conductor_layers_m':'KPLC25','geometry.conductor_lateral_offsets_m':'KPLC25'}
 for keys,value in walk(scene['joint_reference_profile']):
  if keys[0] in ('provenance','profile_id','role'):continue
  field='.'.join(keys);source_id=inherited.get(field)
  numeric=isinstance(value,(int,float)) and not isinstance(value,bool) or isinstance(value,list) and all(isinstance(x,(int,float)) and not isinstance(x,bool) for x in value)
  kind='declared_reference_assumption' if numeric or isinstance(value,bool) else 'declared_reference_model_or_policy'
  if source_id=='KPLC25':kind='transferred_reference_geometry_not_local_survey'
  elif source_id:kind='manufacturer_specification_or_derived_reference'
  evidence=[]
  if keys[0]=='vehicle':evidence.append('总结/03_实验数据/工程化联合求解/module_validation/road/results.json')
  if keys[0]=='geometry' and keys[-1] in ['target_patch_width_m','target_patch_height_m','target_surface_samples']:
   evidence.append('总结/03_实验数据/工程化联合求解/module_validation/geography/results.json')
  rows.append(dict(id=field,value=value,input_pointer='/joint_reference_profile/'+'/'.join(keys),
    source_kind=kind,source_ids=[source_id] if source_id else [],field_calibrated=False,
    validation_evidence=[bind(ROOT/e) for e in evidence],validation_scope='only the named component checks; no full physics validation',
    role='legacy deterministic reference profile, not automatically the revision8 complete profile'))
 legacy_count=len(rows);profiles={};component_bindings={}
 for module in ['battery','flight','weather','image']:
  path=ROOT/f'Agent/Publication/inspection_reference_{module}_profile.json';profile=read(path)
  evidence_module='capture_replanning' if module=='image' else module
  result_path=ROOT/f'总结/03_实验数据/工程化联合求解/module_validation/{evidence_module}/results.json'
  profiles[module]=bind(path);component_bindings[module]=bind(result_path)
  params=profile.get('parameters')
  if params is None:
   packet=profile['packets'][0]
   params={k:packet[k] for k in ['valid_from_s','valid_until_s','near_ground','visibility_m','minimum_capture_visibility_m','wind_limit_mps','gust10_limit_mps','temperature_range_C','scenarios']}
  for keys,value in walk(params):
   field='.'.join(keys)
   source_kind=profile.get('provenance',{}).get(field,'declared_reference_assumption')
   if module=='image' and field in ['image_width_px','image_height_px','diagonal_fov_deg_at_wide','focal_length_mm_bounds']:
    source_kind='manufacturer_specification'
   rows.append(dict(id='revision8_reference.'+module+'.'+field,value=value,
      input_file=profiles[module],input_pointer=('/parameters/' if 'parameters' in profile else '/packets/0/')+'/'.join(keys),
      source_kind=source_kind,source_ids=[],field_calibrated=False,
      validation_evidence=[component_bindings[module]],validation_scope='declared finite component examples only',
      role='active reference component profile; joint transcription parity remains separate'))
  if module=='flight':
   for field,value in profile.get('contact_parameters',{}).items():
    rows.append(dict(id='revision8_reference.flight.contact.'+field,value=value,
      input_file=profiles[module],input_pointer='/contact_parameters/'+field,
      source_kind='declared_reference_assumption',source_ids=[],field_calibrated=False,
      validation_evidence=[bind(ROOT/'总结/03_实验数据/工程化联合求解/module_validation/platform_contact/results.json'),
                           bind(ROOT/'总结/03_实验数据/工程化联合求解/module_validation/takeoff_landing/results.json')],
      validation_scope='pointwise contact, ground-drive midpoint and calm28s programmed execution only',
      role='reference contact/ground motor parameters; regional executor remains separate'))
 group_module={'target_platform_power':'flight','rotor_inertia_allocation_lag_limits':'flight','landing_contact':'flight',
   'controller_and_tracking':'flight','battery_electrothermal':'battery','charger_controls':'battery',
   'forecast_and_residuals':'weather','near_ground_vertical_wind':'weather','image_acceptance':'image'}
 reference_groups=[dict(id=g,profile=profiles[m],evidence=component_bindings[m],
   binding_kind='declared_reference_component',field_calibrated=False,
   joint_runtime_parity_verified=False) for g,m in group_module.items()]
 for group in reference_groups:
  if group['id']=='landing_contact':
   group['additional_evidence']=[bind(ROOT/'总结/03_实验数据/工程化联合求解/module_validation'/n/'results.json') for n in ['platform_contact','takeoff_landing']]
 rev=contract['integration_revision']
 rev['reference_component_parameter_bindings']=reference_groups
 rev['reference_numeric_component_groups_bound']=len(reference_groups)
 rev['field_calibration_unresolved']=True
 # Original unknown same-platform measurements remain unknown. No gate flag
 # is changed merely because independently validated reference inputs exist.
 for row in rev['sources']:
  if row['id']=='replanning_interface':row['sha256']=bind(ROOT/row['path'])['sha256']
 save(CONTRACT,contract)
 measured=ROOT/'总结/03_实验数据/电池能耗公开机型验证/measured_calibration/power_profile.json'
 source_ids=set(x for r in rows for x in r['source_ids'])
 result=dict(schema_version=1,observed_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
  authoritative_scene=bind(SCENE),authoritative_contract=bind(CONTRACT),parameters=rows,
  source_catalog=[x for x in scene['source_catalog'] if x['id'] in source_ids],
  separate_measured_platform=dict(platform='DJI_Matrice_100',profile=bind(measured),
    may_calibrate_M300=False,reason='different airframe, payload, battery and training domain'),
  reference_component_groups=reference_groups,
  unbound_revision8_groups=[{**x,'reference_component_binding':next(r for r in reference_groups if r['id']==x['id']),
    'next_action':'same-platform calibration remains missing; reference profile is available for joint assembly and parity validation'} for x in contract['integration_revision']['unbound_parameters']],
  missing_field_calibration=contract['missing_required_parameters'],
  summary=dict(parameter_entries=len(rows),source_kind_counts=dict(collections.Counter(r['source_kind'] for r in rows)),
    legacy_parameter_entries=legacy_count,reference_parameter_entries=len(rows)-legacy_count,
    unresolved_field_calibration_groups=len(contract['integration_revision']['unbound_parameters']),
    reference_component_groups_bound=len(reference_groups),full_numeric_instance_bound=False,
    legacy_zero_cooling_is_not_electrothermal_validation=True),
  policy='source-first; no invented measurements; declared assumptions allowed under user authorization, with sensitivity and component evidence')
 output=BOOK/'evidence/current_parameter_ledger.json';save(output,result)
 sources=[ROOT/'Agent/Publication/engineering_inspection_contract.json',ROOT/'Agent/Publication/engineering_inspection_model.tex',
  ROOT/'Agent/Publication/engineering_model_dependencies.py',ROOT/'Agent/Publication/engineering_road_motion.py',ROOT/'Agent/Publication/engineering_regional_structure.py',
  ROOT/'scripts/validate_inspection_geography_module.py',ROOT/'scripts/validate_inspection_road_module.py',ROOT/'scripts/verify_inspection_integrated_definition.py',Path(__file__)]
 sources += [ROOT/f'Agent/Publication/{n}.py' for n in ['engineering_battery_electrothermal','engineering_reference_flight','engineering_reference_weather','engineering_image_acceptance','engineering_replanning','engineering_service_boundary']]
 sources += [ROOT/f'Agent/Publication/{n}.py' for n in ['engineering_joint_transcription','engineering_platform_contact','engineering_joint_charging']]
 sources += [ROOT/f'scripts/validate_inspection_{n}.py' for n in ['battery_module','flight_module','weather_module','capture_replanning','service_boundary']]
 sources += [ROOT/f'scripts/validate_inspection_{n}.py' for n in ['joint_transcription','platform_contact','takeoff_landing','joint_charging']]
 sources += [ROOT/v['path'] for v in profiles.values()]
 save(BOOK/'reproduction/current_source_index.json',dict(observed_at=result['observed_at'],sources=[bind(p) for p in sources],
  workspace=str(ROOT),frozen_reproduction_project='historical source snapshot retained; do not silently relabel as current',
  source_pdf_generated=False,artifact_role='current source pointers and hashes; existing historical electronic snapshot remains unchanged'))
 print(json.dumps(result['summary'],ensure_ascii=False))
if __name__=='__main__':main()
