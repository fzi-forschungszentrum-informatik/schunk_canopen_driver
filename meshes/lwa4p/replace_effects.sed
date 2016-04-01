# This is a helper file that was used to replace material effects to fix some problems with RVIZ. 
# You should not need it but maybe its helpfull some day.
#
# run this with
# sed -i -f replace_effect.sed *.dae
/<library_effects/,/<\/library_effects/d
/<library_images\/>/ {
  r materials.daelib
}
