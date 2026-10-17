"""Loopback-only by default, offline reference browser; never publishes ROS commands."""
import argparse
import io
import json
from pathlib import Path
import tempfile
import zipfile
from fastapi import FastAPI, UploadFile, HTTPException, Form
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from .backend import EditorService

STATIC=Path(__file__).with_name('static')
ROOT=Path(__file__).resolve().parents[3]


def create_app(config_file,output_directory,map_file=None,reference_directory=None,replan_speed=False):
    app=FastAPI(title='AIMSRacer offline reference editor')
    service=EditorService(config_file,output_directory)
    if map_file: service.import_map(map_file)
    initial=service.load_reference(reference_directory,replan_speed) if reference_directory else None
    app.state.service=service
    app.mount('/static',StaticFiles(directory=STATIC),name='static')

    def checked(function,*args):
        try: return function(*args)
        except (ValueError,KeyError,TypeError,OSError) as exc:
            raise HTTPException(422,str(exc)) from exc

    @app.get('/')
    def index(): return FileResponse(STATIC/'index.html')

    @app.get('/api/session')
    def session():
        return dict(map=service.active_map,project=initial,config_file=str(service.config_file),
                    vehicle_geometry={k:getattr(service.config,k) for k in ('wheelbase','rear_offset','half_width','half_length','front_extent','rear_extent')})

    @app.post('/api/map')
    async def map_upload(file:UploadFile):
        with tempfile.NamedTemporaryFile(suffix='.pcd') as temporary:
            while chunk:=await file.read(1024*1024): temporary.write(chunk)
            temporary.flush()
            return checked(service.import_map,temporary.name)

    @app.post('/api/preview')
    def preview(project:dict): return checked(service.preview,project)

    @app.post('/api/export')
    def export(payload:dict):
        return checked(service.export,payload.get('project'),payload.get('name'))

    @app.post('/api/reference')
    async def reference(file:UploadFile,replan_speed:bool=Form(False)):
        content=await file.read()
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)
            try:
                with zipfile.ZipFile(io.BytesIO(content)) as archive:
                    # Only extract known direct files; no archive path is trusted.
                    known={'path.csv','metadata.json','reference.json','project.json','calibration_regions.json'}
                    for name in known:
                        matches=[entry for entry in archive.namelist() if Path(entry).name==name and not entry.endswith('/')]
                        if len(matches)>1: raise ValueError('ambiguous reference archive filenames')
                        if matches: (target/name).write_bytes(archive.read(matches[0]))
                return checked(service.load_reference,target,replan_speed)
            except (zipfile.BadZipFile,ValueError,KeyError) as exc:
                raise HTTPException(422,str(exc)) from exc

    @app.get('/api/bundle/{name}')
    def bundle(name:str):
        checked(service.check_name,name)
        target=service.output_directory/name
        if not (target/'reference.json').is_file(): raise HTTPException(404,'bundle not found')
        stream=io.BytesIO()
        with zipfile.ZipFile(stream,'w',zipfile.ZIP_DEFLATED) as archive:
            for filename in ('path.csv','metadata.json','reference.json','project.json','calibration_regions.json'):
                archive.write(target/filename,filename)
        stream.seek(0)
        return StreamingResponse(stream,media_type='application/zip',headers={'Content-Disposition':f'attachment; filename="{name}.zip"'})

    return app


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(ROOT/'src/controller/config/vehicle.yaml'))
    parser.add_argument('--output',required=True,help='new offline bundles and immutable original map cache')
    parser.add_argument('--map',dest='map_file')
    parser.add_argument('--reference',help='existing prepared reference directory to edit')
    parser.add_argument('--replan-speed',action='store_true',help='explicitly replan an imported reference speed profile using --config')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    import uvicorn
    uvicorn.run(create_app(args.config,args.output,args.map_file,args.reference,args.replan_speed),host=args.host,port=args.port)


if __name__=='__main__': main()
