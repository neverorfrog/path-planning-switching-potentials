clear; close all;

snaptot = imread("simulazioni/treostacoli2/snap1.png");
for i = 1 : 5
    path = sprintf("simulazioni/treostacoli2/snap%d.png", i*20);
    snaptot = addsnaps(snaptot,path);
    alpha(snaptot,0.5);
end
imshow(snaptot)
