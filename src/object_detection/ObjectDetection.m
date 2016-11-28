# To find hard coded value in the code :
# Search for "% hardcoded value"

#17p en x = 1cm
#17p en y = 1cm
#1p = 1/17cm

%clc
%clear all
%close all
%pkg load image # For octave only
function [response] = ObjectDetection (image_file) # Image matrix RGB, .mat PCL file (from CreateMatFile.m)

  modeDebug = 1; % set to 1 to get figures
  %image_file = "example";
  try
    pkg load image
  catch ME
    print("Image pkg already loaded")
  end
  OnepixelInMM = 1/17*10;

  % Index number in order to create new input in list (response)
  n_embout = 0;
  n_puit = 0;
  n_gpuit = 0;
  n_trash = 0;

  response = struct(); % Structure sent to web interface

  name = cellstr(image_file);
  s2 = '.jpg';
  s1 = '.mat';
  FirstTime = 1;
  file_name = strcat(char(name),s2); % Create image name
  matfilename = strcat(char(name),s1); % create .mat file name

  imageA = imread(file_name);  %read image
  B = imrotate(imageA,90); % Rotate 90 image
  % Flag to determine if there is something detected in the image
  emptyPuit = 0;
  emptyEmbout = 0;
  emptyGrospuit = 0;
  emptyPoubelle = 0;
  BBIndex = NaN(4,4); % Matrix of modules (can determine 4 object of each type, detect 4 type of object)

  B2 = rgb2lab(B);

  E = histeq(B2(:,:,1)./max(max(B2(:,:,1))),100);
  % Must do it twice IOT get dark and clear modules
  E1 = entropyfilt(E);
  E2 = entropyfilt(-E);
  figure,imshow(E1) ;
  if modeDebug
    figure
    subplot(1,2,1)
    imshow(E1)
    title('Result from Entropy filter E')
    subplot(1,2,2)
    imshow(E2)
    title('Result from Entropy filter -E')
  end
  Eim1 = mat2gray(E1);
  Eim2 = mat2gray(E2);
  if modeDebug
    figure
    subplot(1,2,1)
    imshow(Eim1);
    subplot(1,2,2)
    imshow(Eim2)
  end
  BW1 = im2bw(Eim1,graythresh(Eim1)); # adaptivethreshold more powerful than graythresh (for python conversion)
  BW2 = im2bw(Eim2,graythresh(Eim2));
  BW3 = BW1 | BW2;
  if modeDebug
    figure
    subplot(3,1,1)
    imshow(BW1);
    subplot(3,1,2)
    imshow(BW2)
    subplot(3,1,3)
    imshow(BW3)
    title('OR operation from preceding images')
  end
  BWao = bwareaopen(BW3,3000); %pt 2000
  if modeDebug
    figure,imshow(BWao)
    title('Removed object with less than 1000 elements fron bin image')
  end
  nhood = true(9);
  closeBWao = imclose(BWao,nhood);
  if modeDebug
    figure
    imshow(closeBWao)
    title('Result of dilatation and erosion of BWao')
  end
  fillBWao = imfill(closeBWao,'holes');
  if modeDebug
    figure,imshow(fillBWao); hold on
  end
  labels = bwlabel(fillBWao);

  RP = regionprops(labels,'MajorAxisLength','MinorAxisLength','BoundingBox');
  BoundingBox = cat(1,RP.BoundingBox);
  MajorAxis = cat(1,RP.MajorAxisLength);
  MinorAxis = cat(1,RP.MinorAxisLength);

  %-------------------------------------------------------------------------
  % Transformation from 3D matrix to 2D IOT get mean height of squared region (above modules)
  #matfilename = 'example.mat'
  object_B = load(matfilename);
  x = object_B.x;
  y = object_B.y;
  z = object_B.z;
  theta = 5;
  matrix_Ry = [cosd(theta) 0 sind(theta); 0 1 0; -sind(theta) 0 cosd(theta)];
  % Application of matrix on each x y z elements
  for n=1:length(x)
      matrix_m = [x(n) y(n) z(n)];
      result_m = matrix_m*matrix_Ry;
      x_res(n) = result_m(1);
      y_res(n) = result_m(2);
      z_res(n) = result_m(3);
  end
  % Deleting the floor (Substract 3D data)
  max_val = 0.345; % hardcoded value
  ind = find(abs(x_res)<=0.0);  %0.46 = embout
  x_res(ind) = NaN(size(ind));
  ind = find(abs(x_res)>=max_val);  %0.46 = embout
  x_res(ind) = NaN(size(ind));
  xbob = x_res';
  zbob = z_res';
  ybob = y_res';
  % For trash only - Deleting point being a lot higher-------------------
  ind = find(abs(x_res)>=0.26); % hardcoded value
  x_res(ind) = NaN(size(ind));
  xbob_poubelle = x_res';
  % ---------------------------------------------------------------------
  % Bringing back 3D image to a 2D representation. This allows us to work
  % more easily with the 2D image and the PCL because coordinnates or now
  % similar.
  A = [0 1 0 -0.5;0 0 1 -0.5; 1 -0 0 -1.366025403784439;-1.414213562373095 0 0 2.931851652578136];
  [m,n] = size(xbob);
  x4d = [xbob(:),ybob(:),zbob(:),ones(m*n,1)]';
  x2d = A*x4d;
  x2 = zeros(m,n);
  y2 = zeros(m,n);
  x2(:) = x2d(1,:);
  y2(:) = x2d(2,:);

  % - Same as above, but for the trash ----------------------------------
  [m2,n2] = size(xbob_poubelle);
  x4d_2 = [xbob_poubelle(:),ybob(:),zbob(:),ones(m2*n2,1)]';
  x2d2 = A*x4d_2;
  x22 = zeros(m2,n2);
  x22(:) = x2d2(1,:);
  % ---------------------------------------------------------------------
  % We delete data that is too high (noise)
  ind = find((abs(x2)) >= 2.5);
  x2(ind) = NaN;
  ind = find(y2 <= -2.5);
  y2(ind) = NaN;
  %----------------------------------------------------------------------
  % Reshape de PCL to fit the 2D image
  image_size = size(fillBWao);
              % x2 - min(x2) -- to left translate all values
          %  * 500/max_value (to multiply all value so they are
          %  distributed along x axis equally)
  xaxis_PCL = (abs(x2) - min(abs(x2))).*image_size(2)/max(abs(x2) - min(abs(x2)));
  yaxis_PCL = (abs(y2) - min(abs(y2))).*image_size(1)/max(abs(y2) - min(abs(y2)));

   % poubelle -----------------------------------------------------------
   % Recompute xaxis_PCL but for the trash
  ind = find((abs(x22)) >= 2.5);
  x22(ind) = NaN;
  xaxis_PCL2 = (abs(x22) - min(abs(x2))).*image_size(2)/max(abs(x2) - min(abs(x2))); % Need the same transform as x2,
  % Otherwise it's causing the PCL to stretch all along the image, which
  % is not wanted
   % --------------------------------------------------------------------
  if modeDebug
    figure; imshow(fillBWao)
    hold on
  end

  for i=1:size(BoundingBox)
      % Hardcoded value: all those -> until end
      if modeDebug
        rectangle('Position',BoundingBox(i,:),'EdgeColor','b','LineWidth',3)
      end
      % Values of each modules for longest and shortest axes
      indmax_embout = find(MajorAxis>=220 & MajorAxis<=470); %doubleEmbout
      indmin_embout = find(MinorAxis>=120 & MinorAxis<=260); %doubleEmbout >=222
      indmax_puit = find(MajorAxis>=200 & MajorAxis<=470);   %mixed       & puit norm
      indmin_puit = find(MinorAxis>=135 & MinorAxis<=190);   %mixed       & puit norm
      indmax_grosPuit = find(MajorAxis>=300 & MajorAxis<=450);
      indmin_grosPuit = find(MinorAxis>=115 & MinorAxis<=190);
      indmax_poubelle = find(MajorAxis>=500 & MajorAxis<=630);
      indmin_poubelle = find(MinorAxis>=230 & MinorAxis<=340);



  end
  n = 1; % BBIndex index % Hardcoded value
  if isfinite(indmax_puit)
      try
          indexes_puit = sort([indmax_puit; indmin_puit]);
          similar_indexes = find(diff(indexes_puit(:,1))==0);
  %        if length(indmax_puit) > length(indmin_puit)
  %          dump_mat = nan(1,length(indmax_puit));
  %          indmin_puit = [dump_mat(1:(length(dump_mat)-length(indmin_puit')),1)' indmin_puit']';
  %        elseif length(indmin_puit) > length(indmax_puit)
  %          dump_mat = nan(length(indmin_puit),1);
  %          indmax_puit = [dump_mat(1:(length(dump_mat)-length(indmax_puit')),1)' indmax_puit']';
  %        elseif length(indmax_puit) == length(indmin_puit)
  %          indmax_puit = indmax_puit;
  %        end

  %        index = sort(indmax_puit) == sort(indmin_puit);
  %        l_index = length(indmin_puit(index(find(index>0))));
          l_index = length(similar_indexes);
          BBIndex(1:l_index,n) = indexes_puit(similar_indexes);
  %        BBIndex(1:l_index,n) = indmin_puit(index);
          if modeDebug
            disp('May have found a well')
          end

      catch Me
          if modeDebug
            disp('Well indexes does not match')
          end
      end

  else
      emptyPuit = 1;
  end
  if isfinite(indmax_embout)
      try
          indexes_embout = sort([indmax_embout; indmin_embout]);
          similar_indexes = find(diff(indexes_embout(:,1))==0);
  %        if length(indmax_embout) > length(indmin_embout)
  %          dump_mat = nan(1,length(indmax_embout));
  %          indmin_embout = [dump_mat(1:(length(dump_mat)-length(indmin_embout')),1) indmin_embout']';
  %        elseif length(indmin_embout) > length(indmax_embout)
  %          dump_mat = nan(length(indmin_embout),1);
  %          indmax_embout = [dump_mat(1:(length(dump_mat)-length(indmax_embout')),1) indmax_embout']';
  %        elseif length(indmax_embout) == length(indmin_embout)
  %          indmax_embout = indmax_embout;
  %        end
          l_index = length(similar_indexes);
          BBIndex(1:l_index,n+1) = indexes_embout(similar_indexes);
  %        index = sort(indmax_embout) == sort(indmin_embout);
  %        l_index = length(indmin_embout(index(find(index>0))));
  %        BBIndex(1:l_index,n+1) = indmax_embout(index);
          if modeDebug
            disp('May have found tip holder')
          end
      catch Me
          if modeDebug
            disp('Tip holder indexes does not match')
          end
      end
  else
      emptyEmbout = 1;
  end

  if isfinite(indmax_grosPuit)
      try
  %        if length(indmax_grosPuit) > length(indmin_grosPuit)
  %          dump_mat = nan(1,length(indmax_grosPuit));
  %          indmin_grosPuit = [dump_mat(1:(length(dump_mat)-length(indmin_grosPuit')),1) indmin_grosPuit']';
  %        elseif length(indmin_grosPuit) > length(indmax_grosPuit)
  %          dump_mat = nan(length(indmin_grosPuit),1);
  %          indmax_grosPuit = [dump_mat(1:(length(dump_mat)-length(indmax_grosPuit')),1) indmax_grosPuit']';
  %        elseif length(indmax_grosPuit) == length(indmin_grosPuit)
  %          indmax_grosPuit = indmax_grosPuit;
  %        end
          indexes_grosPuit = sort([indmax_grosPuit; indmin_grosPuit]);
          similar_indexes = find(diff(indexes_grosPuit(:,1))==0);
          l_index = length(similar_indexes);
          BBIndex(1:l_index,n+2) = indexes_grosPuit(similar_indexes);
  %        index = sort(indmax_grosPuit) == sort(indmin_grosPuit);
  %        l_index = length(indmin_grosPuit(index));
  %        BBIndex(1:l_index,n+2) = indmin_grosPuit(index);
          if modeDebug
            disp('May have found a large well')
          end
      catch Me
        if modeDebug
          disp('Big well indexes does not match')
        end
      end

  else
      emptyGrospuit = 1;
  end

  if isfinite(indmax_poubelle)
      try
  %        if length(indmax_poubelle) > length(indmin_poubelle)
  %          dump_mat = nan(1,length(indmax_poubelle));
  %          indmin_poubelle = [dump_mat(1,1:(length(dump_mat)-length(indmin_poubelle'))) indmin_poubelle']';
  %        elseif length(indmin_poubelle) > length(indmax_poubelle)
  %          dump_mat = nan(length(indmin_poubelle),1);
  %          indmax_poubelle = [dump_mat(1,1:(length(dump_mat)-length(indmax_poubelle'))) indmax_poubelle']';
  %        elseif length(indmax_poubelle) == length(indmin_poubelle)
  %          indmax_poubelle = indmax_poubelle;
  %        end
          indexes_poubelle = sort([indmax_poubelle; indmin_poubelle]);
          similar_indexes = find(diff(indexes_poubelle(:,1))==0);
          l_index = length(similar_indexes);
          BBIndex(1:l_index,n+3) = indexes_poubelle(similar_indexes);

  %        index = sort(indmax_poubelle) == sort(indmin_poubelle);
  %        l_index = length(indmin_poubelle(index));
  %        BBIndex(1:l_index,n+3) = indmin_poubelle(index); % Il n'y aura jamais plus d'une poubelle
          if modeDebug
            disp('May have found a trash')
          end
      catch Me
          if modeDebug
            disp('trash indexes does not match')
          end
      end

  else
      emptyPoubelle = 1;
  end

  if emptyPuit == 1 && emptyEmbout == 1 && emptyGrospuit == 1 && emptyPoubelle == 1; % True = Nothing detected in the picture
      if modeDebug
        disp('!Nothing detected!')
      end
      %continue; % Do the next iteration
  end
  if modeDebug
    figure
    imshow(B) % mask
    hold on
  end


  for n = 1:max(find(BBIndex(1,:) > 0)) %--------------------------------
      for i = 1:max(find(BBIndex(:,n) > 0))
          % These arrays can be modified or overwritten by subsequent
          % operation. This is why they are re-initialized here.
          xaxis_PCL_mod = xaxis_PCL;
          yaxis_PCL_mod = yaxis_PCL;

          try
              % We look at points in the BoundingBox only, not the other
              % scattered all around.
              ind = find(xaxis_PCL_mod >= BoundingBox(BBIndex(i,n),1) & xaxis_PCL_mod <= BoundingBox(BBIndex(i,n),1)+BoundingBox(BBIndex(i,n),3) & yaxis_PCL_mod >= BoundingBox(BBIndex(i,n),2) & yaxis_PCL_mod <= BoundingBox(BBIndex(i,n),2)+BoundingBox(BBIndex(i,n),4));
              if modeDebug
                rectangle('Position',BoundingBox(BBIndex(i,n),:),'EdgeColor','b','LineWidth',3)
                disp(['Draw rectangle :',num2str(BBIndex(i,n))])
                disp([('ind found :'),num2str(size(ind))])
              end
              % The indexes not inside the box/rectangle are set to NaN
              FullIdx=1:length(xaxis_PCL_mod);
              NotInd=setdiff(FullIdx,ind);
              xaxis_PCL_mod(NotInd) = NaN; % Other values not in BoundingBox are set to NaN (so not represented in the figure)
              yaxis_PCL_mod(NotInd) = NaN;

              try
                % ------- Trash--- --------------------------------------------
                % We look at points in the BoundingBox only, not the other
                % scattered all around.
                ind = find(xaxis_PCL2 >= BoundingBox(BBIndex(i,4),1) & xaxis_PCL2 <= BoundingBox(BBIndex(i,4),1)+BoundingBox(BBIndex(i,4),3) & yaxis_PCL >= BoundingBox(BBIndex(i,4),2) & yaxis_PCL <= BoundingBox(BBIndex(i,4),2)+BoundingBox(BBIndex(i,4),4));
                FullIdx=1:length(xaxis_PCL2);
                NotIdx=ind;
                ind=setdiff(FullIdx,NotIdx);
                xaxis_PCL2(ind) = NaN;
              if modeDebug
                scatter(xaxis_PCL2,yaxis_PCL_mod,'c') % Plot data found (ideally the top
                % part of the garbage box
              end
              catch ME
                  disp('No garbage detected')
              end

              k=sum(sum(~isnan(xaxis_PCL_mod),2)); % We count how much data in the bounding box are not NaN
              k_poubelle = sum(sum(~isnan(xaxis_PCL2),2));
              if modeDebug
                disp([('k value: '),num2str(k)])
                disp([('k trash: '),num2str(k_poubelle)])
              end
              % IF more than 1000 point AND EMBOUT size corresponds, then:
              if (k >=400) && BBIndex(i,2)>0
                  x_value = num2str(BoundingBox(BBIndex(i,n),1)*OnepixelInMM);
                  y_value = num2str(BoundingBox(BBIndex(i,n),2)*OnepixelInMM);
                  z_value = num2str(min(xbob)) # Value is not in mm  (To understand look at PCL/scatter3 plot)

                  % We crop 10% around the rectangle representing the detected module
                  % in order to send that picture to the web interface.
                  x_initial = BoundingBox(BBIndex(i,n),1);
                  x_initial = x_initial-0.1*x_initial;
                  y_initial = BoundingBox(BBIndex(i,n),2);
                  y_initial = y_initial-0.1*y_initial;
                  x_final = BoundingBox(BBIndex(i,n),3);
                  x_final = x_final*1.1;
                  y_final = BoundingBox(BBIndex(i,n),4);
                  y_final = y_final*1.1;
                  Idump = imcrop(B,[x_initial y_initial x_final y_final]);

                  structname = strcat(char("tip_holder"),num2str(n_embout));
                  response.(structname) = {"tip_holder",structname,Idump, x_value,y_value,z_value}; # ID x_pos[mm], y_pos[mm], z_pos(invalid)
                  n_embout = n_embout+1;
                  if modeDebug
                    strmax = ['tip_holder ',x_value,' / ',y_value ,' mm'];
                    text((BoundingBox(BBIndex(i,n),1)),(BoundingBox(BBIndex(i,n),2)+10),strmax,'HorizontalAlignment','left','color','green','BackgroundColor','k','FontSize',15);
                    scatter(xaxis_PCL_mod,yaxis_PCL_mod,'g')
                    %figure
                    %imshow(Idump)
                    %title('Croped image from Embout')
                    %response
                  end
              end
              % IF less than 1000 point AND PUIT size corresponds, then:
              if k<=100 && BBIndex(i,1)>0
                  x_value = num2str(BoundingBox(BBIndex(i,n),1)*OnepixelInMM);
                  y_value = num2str(BoundingBox(BBIndex(i,n),2)*OnepixelInMM);
                  z_value = num2str(min(xbob));

                  % We crop 10% around the rectangle representing the detected module
                  % in order to send that picture to the web interface.
                  x_initial = BoundingBox(BBIndex(i,n),1);
                  x_initial = x_initial-0.1*x_initial;
                  y_initial = BoundingBox(BBIndex(i,n),2);
                  y_initial = y_initial-0.1*y_initial;
                  x_final = BoundingBox(BBIndex(i,n),3);
                  x_final = x_final*1.1;
                  y_final = BoundingBox(BBIndex(i,n),4);
                  y_final = y_final*1.1;
                  Idump = imcrop(B,[x_initial y_initial x_final y_final]);

                  structname = strcat(char("well"),num2str(n_puit));
                  response.(structname) = {"well", structname,Idump,x_value,y_value,z_value}; # ID x_pos[mm], y_pos[mm], z_pos(invalid)
                  n_puit = n_puit+1;
                  if modeDebug
                    strmax = ['Well ',x_value,' / ',y_value ,' mm'];
                    text((BoundingBox(BBIndex(i,n),1)),(BoundingBox(BBIndex(i,n),2)+10),strmax,'HorizontalAlignment','left','color','red','BackgroundColor','k','FontSize',15);
                    scatter(xaxis_PCL_mod,yaxis_PCL_mod,'r')
                    %figure
                    %imshow(Idump)
                    %title('Croped image from Well')
                  end
              end
              % IF more than 800 point AND Large well size corresponds, then:
              if k>=100 && BBIndex(i,3)>0
                  x_value = num2str(BoundingBox(BBIndex(i,n),1)*OnepixelInMM);
                  y_value = num2str(BoundingBox(BBIndex(i,n),2)*OnepixelInMM);
                  z_value = num2str(min(xbob));
                  strmax = ['Large Well ',x_value,' / ',y_value,' mm' ];
                  scatter(xaxis_PCL_mod,yaxis_PCL_mod,'b')
                  text((BoundingBox(BBIndex(i,n),1)),(BoundingBox(BBIndex(i,n),2)+10),strmax,'HorizontalAlignment','left','color','red','BackgroundColor','k','FontSize',15);

                  % We crop 10% around the rectangle representing the detected module
                  % in order to send that picture to the web interface.
                  x_initial = BoundingBox(BBIndex(i,n),1);
                  x_initial = x_initial-0.1*x_initial;
                  y_initial = BoundingBox(BBIndex(i,n),2);
                  y_initial = y_initial-0.1*y_initial;
                  x_final = BoundingBox(BBIndex(i,n),3);
                  x_final = x_final*1.1;
                  y_final = BoundingBox(BBIndex(i,n),4);
                  y_final = y_final*1.1;
                  Idump = imcrop(B,[x_initial y_initial x_final y_final]);

                  structname = strcat(char("LargeWell"),num2str(n_gpuit));
                  response.(structname) = {"LargeWell",structname, Idump,x_value,y_value,z_value}; # ID x_pos[mm], y_pos[mm], z_pos(invalid)
                  n_gp = n_gpuit+1;
                  if modeDebug
                    strmax = ['Large Well',x_value,' / ',y_value,' mm' ];
                    scatter(xaxis_PCL_mod,yaxis_PCL_mod,'b')
                    text((BoundingBox(BBIndex(i,n),1)),(BoundingBox(BBIndex(i,n),2)+10),strmax,'HorizontalAlignment','left','color','red','BackgroundColor','k','FontSize',15);

                    %figure
                    %imshow(Idump)
                    %title('Croped image from Large well')
                  end
              end
              % IF more than 1000 point AND TRASH size corresponds, then:
              if k_poubelle>=50 && BBIndex(i,4)>0 && k>=4000
                  x_value = num2str(BoundingBox(BBIndex(i,n),1)*OnepixelInMM);
                  y_value = num2str(BoundingBox(BBIndex(i,n),2)*OnepixelInMM);
                  z_value = num2str(min(xbob));

                  % We crop 10% around the rectangle representing the detected module
                  % in order to send that picture to the web interface.
                  x_initial = BoundingBox(BBIndex(i,n),1);
                  x_initial = x_initial-0.1*x_initial;
                  y_initial = BoundingBox(BBIndex(i,n),2);
                  y_initial = y_initial-0.1*y_initial;
                  x_final = BoundingBox(BBIndex(i,n),3);
                  x_final = x_final*1.1;
                  y_final = BoundingBox(BBIndex(i,n),4);
                  y_final = y_final*1.1;
                  Idump = imcrop(B,[x_initial y_initial x_final y_final]);
                  structname = strcat(char("trash"),num2str(n_trash));
                  response.(structname) = {"trash",structname, Idump,x_value,y_value,z_value}; # ID x_pos[mm], y_pos[mm], z_pos(invalid)
                  n_trash = n_trash+1;
                  if modeDebug
                    strmax = ['Poubelle ',x_value,' / ',y_value ,' mm'];
                    text((BoundingBox(BBIndex(i,n),1)),(BoundingBox(BBIndex(i,n),2)+10),strmax,'HorizontalAlignment','left','color','c','BackgroundColor','k','FontSize',15);
                    scatter(xaxis_PCL_mod, yaxis_PCL_mod,'r')
                    scatter(xaxis_PCL2,yaxis_PCL_mod,'c')

                    %figure
                    %imshow(Idump)
                    %title('Croped image from trash')
                  end
              end
          catch ME
              disp('No able to find corresponding module')
          end
      end
  end

  % We look at the response structure and make sure each elements are unique.
  % There are some cases where the same elements can appear twice into the structure due
  % to the well and tip_holder very similar size.
  try
    fields = fieldnames(response);
    dump_x = [];
    dump_y = [];
    dump_z = [];
    for i = 1:numel(fields)
      dump_x = [dump_x response.(fields{i})(3)];
      dump_y = [dump_y response.(fields{i})(4)];
      dump_z = [dump_z response.(fields{i})(5)];
    end

    for i = 1:length(dump_x)
      for n = 1:length(dump_x)
        bob(i,n) = isequal(dump_x(i),dump_x(n));
        if i == n
          bob(i,n) = 0;
        end
      end
    end
    [x y] = find(bob == 1);
    for i = 1:length(x)/2
      i = x(i);
      response.(fields{i-1}) = nan;
    end
    catch Me
      print("Unable to process response fields")
  end

  % Output 3D plot IOT compare with 2D pictures and make
  % sure the relevant PCL has been used for the detection
  if modeDebug
    figure;scatter3(xbob,ybob,zbob)
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
  end

endfunction
