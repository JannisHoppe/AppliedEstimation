function coords = extract_landmark_coords(filename)

d = load(filename);
if nargin < 1
    disp('You need to supply the file with landmarks ')
    return
end

coords = [d(:,2), d(:,3)];
