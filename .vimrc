set makeprg=[[\ -f\ Makefile\ ]]\ &&\ make\ \\\|\\\|\ make\ -C\ build/

set grepprg=grep\ -nrI\ --exclude-dir={.git,docs,build,libraries}\ --exclude=*tags

" recreate tags file with F5
map <F5> :GenerateSpaintTags<CR>

set tags+=spaint.tags
set tags+=library.tags
set tags+=infinitam.tags

function GenerateSpaintTags()
  !ctags -R --sort=yes --c++-kinds=+p --fields=+iaS --extra=+qf --exclude=libraries -f spaint.tags .
endfunction
command GenerateSpaintTags execute GenerateSpaintTags()

function GenerateLibraryTags()
  !ctags -R --sort=yes --c++-kinds=+p --fields=+iaS --extra=+qf -L libraries/ -f library.tags .
endfunction
command GenerateLibraryTags execute GenerateLibraryTags()

function GenerateInfiniTAMTags()
  !ctags -R --sort=yes --c++-kinds=+p --fields=+iaS --extra=+qf -L ../InfiniTAM/ -f infinitam.tags .
endfunction
command GenerateInfiniTAMTags execute GenerateInfiniTAMTags()
