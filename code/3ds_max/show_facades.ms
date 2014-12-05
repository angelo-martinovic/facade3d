rollout Fac "Facade" width:185 height:150
(
	
	button btn_run				"Run" pos:[10,20] width:170 height:20
	editText edt_main_path 	"path:" pos:[15,50] width:160 height:16
	
	checkbox chk_export     "export2max" pos:[15,70] width:90 height:16
	checkbox chk_render     "render" pos:[15,90] width:90 height:16
	

	
	on Fac open do
	(
		edt_main_path.text = "..\output/export/3D_3L_facades_3ds";		
		edt_main_path.text = "C:\Users\vop\Dropbox\devel\maxscript\data_city_monge\separ_facades\\";
		
		chk_export.checked = false;   --- if export max file
		chk_render.checked = true;    --- if render images
	)

	
	----------------------------------------------------------------------------------------------
	----------------------------------------------------------------------------------------------
	----------------------------------------------------------------------------------------------






-- RUN IT
	on btn_run pressed do
	(
		--path_scripts = "C:\Users\vop\Dropbox\devel\maxscript";
		for num = 1 to 30  do (
			--- path to the read model if it does not exist, go to next facade...
			path_i = edt_main_path.text+"\\"+num as string
			format "---- file %" path_i
			if (getfiles (path_i+".obj")).count == 0 do (
				format " does not exist..\n"
				continue
			)
			
			--- import some stuff
			ImportFile (path_i+".obj") #noPrompt
			$default.name = "fac";
			
			ImportFile (path_i+"_balc.obj") #noPrompt
		    $default.name = "balc";
			in_file = fopen  (path_i+"_wallWin.boxesbin") "rb"
			if in_file != undefined then (
				dim1 = ReadFloat  in_file
				dim2  = ReadFloat  in_file
				--- read wall
				px = ReadFloat  in_file; py = ReadFloat  in_file;pz = ReadFloat  in_file;w = ReadFloat  in_file;h = ReadFloat  in_file;d = ReadFloat  in_file;
				mybox = box length:d width:w height:h
				$Box01.pivot = $Box01.center;
				$Box01.name = "wall";
				$wall.pos =  [px,-pz,py];
				wall_width = $wall.width;
				
				for b = 1 to dim2-1 do (
					px = ReadFloat  in_file
					py = ReadFloat  in_file
					pz = ReadFloat  in_file
					w = ReadFloat  in_file
					h = ReadFloat  in_file
					d = ReadFloat  in_file
					mybox = box length:d width:w height:h
					$Box01.pivot = $Box01.center
				    $Box01.pos = [px,-pz,py];
					--format "[% % %]\n" px -pz py
					--format " % %  \n" $Box01.pos[1] ($wall.width/2+$wall.pos[1])
					
					----- boolean if it is above the wall
					if (-wall_width/2+$wall.pos[1])>($Box01.pos[1]) then (
						$Box01.pos = [$Box01.pos[1],$Box01.pos[2]+0.3,$Box01.pos[3]];
						$Box01.length = $Box01.length+0.6;
						meshop.attach $fac $Box01
					)else( --- dont use it if they are above the balconies
						boolObj.createBooleanObject $wall
						boolObj.setBoolOp $wall 3
						boolObj.SetOperandB $wall $Box01 4 3
					)
				)
			)
			meshop.attach $fac $wall
			meshop.attach $fac $balc
			format "% \n" path_i
	
			-- UVW map
			m = uvwmap()
			m.maptype = 0 --- planar
			m.axis = 1 --- Y align
			addmodifier $fac (m)
	
			--- material
			mat_path = edt_main_path.text+num as string+".png"
			meditMaterials[10].diffuseMap = Bitmaptexture fileName:mat_path
			$fac.material = meditMaterials[10];
	
			--- render
			if chk_render.checked then (
				path_save_im = edt_main_path.text+num as string+".jpg"
				Render outputfile:path_save_im
				format "  render saved to %\n" path_save_im
			)
			if chk_export.checked then (
			--- export result :)
				select $fac
				saveNodes $fac (path_i+"_export.max")
				format "   export to : %_export.max\n" path_i
			)
			delete $fac
		)
	)	

	
	
	
)
	
	

CreateDialog Fac 
