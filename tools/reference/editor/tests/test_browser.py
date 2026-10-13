"""A real Chromium workflow, all runtime requests confined to loopback."""
import hashlib
import json
import zipfile
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.request
import numpy as np
from pypcd4 import PointCloud
from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[4]


def test_browser_edit_preview_save_reopen_and_export(tmp_path):
    pcd=tmp_path/'map.pcd'
    PointCloud.from_xyz_points(np.array([[-5,-5,0],[5,5,0],[0,0,1]],dtype=np.float32)).save(pcd)
    sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
    environment=dict(os.environ,PYTHONPATH=str(ROOT/'src/controller')+':'+str(ROOT))
    process=subprocess.Popen([sys.executable,'-m','tools.reference.editor.server','--output',str(tmp_path/'output'),'--port',str(port)],cwd=ROOT,env=environment,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    try:
        url=f'http://127.0.0.1:{port}'
        for _ in range(100):
            try: urllib.request.urlopen(url,timeout=.2);break
            except OSError: time.sleep(.1)
        with sync_playwright() as playwright:
            browser=playwright.chromium.launch(headless=True,args=['--enable-webgl','--use-gl=angle','--use-angle=swiftshader','--no-sandbox'])
            page=browser.new_page(viewport={'width':1360,'height':1000},accept_downloads=True)
            errors=[];external=[]
            page.on('pageerror',lambda error:errors.append(str(error)))
            page.on('request',lambda request:external.append(request.url) if not request.url.startswith(url) and not request.url.startswith('blob:') else None)
            page.goto(url)
            page.locator('#map-file').set_input_files(pcd)
            page.wait_for_function("document.querySelector('#map-info').textContent.includes('SHA')")
            bounds=page.locator('#viewport canvas').bounding_box()
            x=bounds['x']+bounds['width']/2;y=bounds['y']+bounds['height']/2
            radius=min(bounds['width'],bounds['height'])*.28
            for i in range(12):
                angle=2*math.pi*i/12
                page.mouse.click(x+radius*math.cos(angle),y-radius*math.sin(angle))
            assert page.locator('#points-info').inner_text().startswith('12 waypoints')
            page.locator('#preview').click()
            page.wait_for_function("document.querySelector('#status').textContent.includes('Valid reference')")
            # Insert/delete use the drawn curve's first control chord midpoint.
            page.locator('#insert').click();page.mouse.click(x+radius*(1+math.cos(math.pi/6))/2,y-radius*math.sin(math.pi/6)/2)
            assert page.locator('#points-info').inner_text().startswith('13 waypoints')
            page.locator('#undo').click();assert page.locator('#points-info').inner_text().startswith('12 waypoints')
            page.locator('#redo').click();assert page.locator('#points-info').inner_text().startswith('13 waypoints')
            page.locator('#delete').click();page.mouse.click(x+radius*(1+math.cos(math.pi/6))/2,y-radius*math.sin(math.pi/6)/2)
            assert page.locator('#points-info').inner_text().startswith('12 waypoints')
            page.locator('#move').click();page.mouse.move(x+radius,y);page.mouse.down();page.mouse.move(x+radius+5,y,steps=5);page.mouse.up()
            page.locator('#undo').click()
            page.locator('#reverse').click();page.locator('#undo').click()
            page.locator('#move').click();page.mouse.move(x,y-radius);page.mouse.down();page.mouse.move(x+5,y-radius,steps=3);page.mouse.up()
            page.locator('#start').click();page.locator('#undo').click();page.locator('#undo').click()
            page.locator('#preview').click();page.wait_for_function("document.querySelector('#status').textContent.includes('Valid reference')")
            page.locator('#mark').click();page.mouse.click(x+radius,y);page.mouse.click(x,y-radius)
            page.locator('#region-add').click();assert page.locator('#regions li').count()==1
            page.locator('#region-add').click();assert page.locator('#regions li').count()==1
            assert 'overlap' in page.locator('#status').inner_text()
            page.locator('#z-min').fill('.5');page.locator('#z-min').press('Tab')
            assert '1 visible' in page.locator('#map-info').inner_text()
            page.locator('#z-min').fill('-100');page.locator('#z-min').press('Tab')
            page.locator('#view-mode').click();assert '3D' in page.locator('#view-label').inner_text()
            page.locator('#view-mode').click()
            with page.expect_download() as saved: page.locator('#save-project').click()
            project_file=tmp_path/'project.json';saved.value.save_as(project_file)
            project=json.loads(project_file.read_text());assert len(project['waypoints'])==12
            page.locator('#clear').click();assert page.locator('#points-info').inner_text().startswith('0 waypoints')
            page.locator('#project-file').set_input_files(project_file);assert page.locator('#points-info').inner_text().startswith('12 waypoints')
            pending=[]
            def delay_export(route):
                pending.append((route,route.fetch()))
            page.route('**/api/export',delay_export)
            page.locator('#bundle-name').fill('stale-lap')
            with page.expect_request('**/api/export'):
                page.locator('#export').click()
            for _ in range(100):
                if pending:break
                page.wait_for_timeout(20)
            assert pending,'export request did not reach delayed response route'
            page.locator('#clear').click()
            with page.expect_response('**/api/export'):
                pending[0][0].fulfill(response=pending[0][1])
            page.wait_for_timeout(100)
            assert page.locator('#download').is_hidden(),'stale export was presented as the current project'
            assert 'Exported stale-lap' not in page.locator('#status').inner_text()
            assert 'Preview needs validation' in page.locator('#metrics').inner_text()
            page.unroute('**/api/export',delay_export)
            page.locator('#project-file').set_input_files(project_file)
            page.locator('#bundle-name').fill('browser-lap');page.locator('#export').click()
            page.locator('#download').wait_for(state='visible')
            with page.expect_download() as bundle: page.locator('#download').click()
            bundle_file=tmp_path/'reference.zip';bundle.value.save_as(bundle_file)
            page.locator('#reference-file').set_input_files(bundle_file)
            page.wait_for_function("document.querySelector('#status').textContent.includes('Reference loaded')")
            changed=tmp_path/'changed-speed.zip'
            with zipfile.ZipFile(bundle_file) as source:
                files={name:source.read(name) for name in source.namelist()}
            frozen=json.loads(files['reference.json']);frozen['speed_profile']['speeds']=[.5]*len(frozen['speed_profile']['speeds'])
            files['reference.json']=(json.dumps(frozen,sort_keys=True,indent=2)+'\n').encode()
            regions=json.loads(files['calibration_regions.json']);regions['reference_sha256']=hashlib.sha256(files['reference.json']).hexdigest()
            files['calibration_regions.json']=json.dumps(regions).encode()
            with zipfile.ZipFile(changed,'w') as target:
                for name,data in files.items():target.writestr(name,data)
            page.locator('#reference-file').set_input_files(changed)
            page.wait_for_function("document.querySelector('#status').textContent.includes('Replan speed')")
            page.locator('#replan-speed').check();page.locator('#reference-file').set_input_files(changed)
            page.wait_for_function("document.querySelector('#status').textContent.includes('Reference loaded')")
            with page.expect_download() as saved:page.locator('#save-project').click()
            evidence_file=tmp_path/'replanned-project.json';saved.value.save_as(evidence_file)
            assert json.loads(evidence_file.read_text())['speed_replan']['prior_reference_sha256']==hashlib.sha256(files['reference.json']).hexdigest()
            assert not errors,errors
            assert not external,external
            screenshot=os.environ.get('EDITOR_SCREENSHOT')
            if screenshot: page.screenshot(path=screenshot,full_page=True)
            real_map=os.environ.get('EDITOR_REAL_PCD')
            if real_map:
                page.locator('#clear').click()
                page.locator('#map-file').set_input_files(real_map)
                page.wait_for_function("document.querySelector('#map-info').textContent.includes('990,482')",timeout=30000)
                real_project=os.environ.get('EDITOR_REAL_PROJECT')
                if real_project:
                    page.locator('#project-file').set_input_files(real_project)
                    page.locator('#preview').click();page.wait_for_function("document.querySelector('#status').textContent.includes('Valid reference')")
                page.locator('aside').evaluate('(element)=>element.scrollTop=0')
                if screenshot:page.screenshot(path=str(Path(screenshot).with_name('editor-real-pcd.png')),full_page=True)
            assert not errors,errors
            assert not external,external
            browser.close()
    finally:
        process.terminate();process.wait(timeout=10)
