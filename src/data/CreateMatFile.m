# Get files from path and output a .mat file
# that has the same name as the image input from
# the Python code
function CreateMatFile(path) %x,y,z,r,g,b

  x = load(strcat(char(path),'/x'));
  y = load(strcat(char(path),'/y'));
  z = load(strcat(char(path),'/z'));
  r = load(strcat(char(path),'/r'));
  g = load(strcat(char(path),'/g'));
  b = load(strcat(char(path),'/b'));
  #name = ['puit_norm'];
  #name = cellstr(imageName);
  #name = strcat(char(name),'.mat');
  save example.mat x y z r g b
endfunction
