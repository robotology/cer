#!/usr/bin/perl -w

# SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

my $verbose_output = 0;

sub print_if_verbose
{
    if ($verbose_output) {
        print (@_);
    }
    return;
}


my @files = `git ls-files src example tests scripts cmake bindings data CMakeLists.txt`;
s{^\s+|\s+$}{}g foreach @files;

open my $handle, '<', "tests/misc/check_license_skip.txt";
chomp(my @skip_files = <$handle>);
close $handle;

my $copyright_iit = "(2006-2021|2006-2022|2006-2023|2023|2023-2023|2024|2024-2024|2025|2025-2025) Istituto Italiano di Tecnologia \\(IIT\\)";
my $copyright_robocub = "2006-2010 RobotCub Consortium";

my $str_bsd3_cpp = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?(
 \\* SPDX-FileCopyrightText: [^\n]+)*
 \\* SPDX-License-Identifier: BSD-3-Clause
 \\*/

END

my $str_bsd3_script = <<END;
^(#!.+

)?(#\\.rst:(
#[^\n]*)+

)?# SPDX-FileCopyrightText: $copyright_iit(
# SPDX-FileCopyrightText: $copyright_robocub)?(
# SPDX-FileCopyrightText: [^\n]+)*
# SPDX-License-Identifier: BSD-3-Clause

END

my $str_bsd3_xml = <<END;
^(<\\?xml[^<]+

)?<!--
  SPDX-FileCopyrightText: $copyright_iit(
  SPDX-FileCopyrightText: $copyright_robocub)?(
  SPDX-FileCopyrightText: [^\n]+)*
  SPDX-License-Identifier: BSD-3-Clause
-->

END

my $str_bsd3_lua = <<END;
^(#!.+

)?-- SPDX-FileCopyrightText: $copyright_iit(
-- SPDX-FileCopyrightText: $copyright_robocub)?(
-- SPDX-FileCopyrightText: [^\n]+)*
-- SPDX-License-Identifier: BSD-3-Clause

END

my $str_bsd3_swig = <<END;
^// SPDX-FileCopyrightText: $copyright_iit(
// SPDX-FileCopyrightText: $copyright_robocub)?(
// SPDX-FileCopyrightText: [^\n]+)*
// SPDX-License-Identifier: BSD-3-Clause

END

my $str_bsd3_mat = <<END;
^% SPDX-FileCopyrightText: $copyright_iit(
% SPDX-FileCopyrightText: $copyright_robocub)?(
% SPDX-FileCopyrightText: [^\n]+)*
% SPDX-License-Identifier: BSD-3-Clause

END

my $str_lgpl2_1_or_later = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?
 \\* SPDX-License-Identifier: LGPL-2.1-or-later
 \\*/

END

my $str_lgpl2_1_or_later_other = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?(
 \\* SPDX-FileCopyrightText: [^\n]+)*
 \\* SPDX-License-Identifier: LGPL-2.1-or-later
 \\*/

END

my $str_gpl2_or_later = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?
 \\* SPDX-License-Identifier: GPL-2.0-or-later
 \\*/

END

my $str_gpl2_or_later_other = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?(
 \\* SPDX-FileCopyrightText: [^\n]+)*
 \\* SPDX-License-Identifier: GPL-2.0-or-later
 \\*/

END

my $str_apache_2 = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit
 \\* SPDX-License-Identifier: Apache-2.0
 \\*/

END

my $str_lgpl2_1_or_later_lua = <<END;
-- SPDX-FileCopyrightText: $copyright_iit(
-- SPDX-FileCopyrightText: $copyright_robocub)?
-- SPDX-License-Identifier: LGPL-2.1-or-later

END

my $str_lgpl2_1_or_later_lua_other = <<END;
-- SPDX-FileCopyrightText: $copyright_iit(
-- SPDX-FileCopyrightText: $copyright_robocub)?(
-- SPDX-FileCopyrightText: [^\n]+)*
-- SPDX-License-Identifier: LGPL-2.1-or-later

END

my $str_gpl3_or_later = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?
 \\* SPDX-License-Identifier: GPL-3.0-or-later
 \\*/

END

my $str_gpl3_or_later_other = <<END;
^/\\*
 \\* SPDX-FileCopyrightText: $copyright_iit(
 \\* SPDX-FileCopyrightText: $copyright_robocub)?(
 \\* SPDX-FileCopyrightText: [^\n]+)*
 \\* SPDX-License-Identifier: GPL-3.0-or-later
 \\*/

END

my $files = 0;
my $ok = 0;
my $skip = 0;
my $errors = 0;
my $known = 0;
my $number_files_marked_to_skip = scalar(@skip_files);
foreach my $skipline (@skip_files) {
    if (substr ($skipline, -1) eq '/') {
    $number_files_marked_to_skip--; }
}

use File::Spec;

foreach my $filename (@files) {
    $files++;

    my $match_filename = 0;
    my $match_dir = 0;
    for (@skip_files) {
        # skip files known to have a broken license
        if ("$filename" eq "$_") {
            $match_filename = 1;
            last;
        }
        # skip directories known to contain files which have a broken license
        my($vol,$dir,$file)    = File::Spec->splitpath($filename);
        my($vol2,$dir2,$file2) = File::Spec->splitpath($_);
        # print "testing $dir $dir2 \n";
        if ("$dir" eq "$_") {
            $match_dir = 1;
            $number_files_marked_to_skip++;
            last;
        }
    }

    if( $match_filename ) {
        print "[SKIP (known file)] $filename\n";
        $known++;
        next;
    }
    elsif( $match_dir ) {
        print "[SKIP (known dir)] $filename\n";
        $known++;
        next;
    }

    # Skip images and binary files
    if ("$filename" =~ /\.(png|svg|jpg|ppm|pgm|bmp|ico|icns)$/) {
        print_if_verbose "[SKIP (image - $1)] $filename\n";
        $skip++;
        next;
    }

    if ("$filename" =~ /\.(avi)$/) {
        print_if_verbose "[SKIP (video - $1)] $filename\n";
        $skip++;
        next;
    }

    if ("$filename" =~ /\.(wav)$/) {
        print_if_verbose "[SKIP (audio - $1)] $filename\n";
        $skip++;
        next;
    }

    if ("$filename" =~ /\.(rar)$/) {
        print_if_verbose "[SKIP (binary - $1)] $filename\n";
        $skip++;
        next;
    }

    if ("$filename" =~ /\.(clang-tidy|clang-format)$/) {
        print_if_verbose "[SKIP (other - $1)] $filename\n";
        $skip++;
        next;
    }


    # Read the file
    my $txt = "";
    open(FIN,"<$filename");
    while (<FIN>) {
        $txt .= $_;
    }
    close(FIN);

    # Skip autogenerated files
    if ("$filename" =~ /\/src_gen\// ||
        "$filename" =~ /\/qtquick2applicationviewer\// ||
        "$filename" =~ /\/idl_generated_code\//) {
        print_if_verbose "[SKIP (autogen)] $filename\n";
        $skip++;
        next;
    }

    # Skip specific files
    if ("$filename" =~ /\/(README|readme)(\.(txt|TXT|md))?$/) {
        print_if_verbose "[SKIP (README)] $filename\n";
        $skip++;
        next;
    }

    if ("$filename" =~ /\/qmldir$/) {
        print_if_verbose "[SKIP (qmldir)] $filename\n";
        $skip++;
        next;
    }

    if ("$filename" =~ /\/Doxyfile(.in)?$/) {
        print_if_verbose "[SKIP (Doxyfile)] $filename\n";
        $skip++;
        next;
    }

    # Skip more files
    if ("$filename" =~ /\.(ui|rc|qrc|xslt|xsl|msg|srv|ini|cfg|toml|txt|TXT|dox|md|plist.in|tex|mdl|map|yaml)$/) {
        if ("$filename" !~ /CMakeLists.txt$/) {
            print_if_verbose "[SKIP (.$1)] $filename\n";
            $skip++;
            next;
        }
    }

    # C++ style BSD-3-Clause
    if ("$txt" =~ /$str_bsd3_cpp/s) {
        if ("$filename" =~ /\.(cpp|cpp.in|c|h|hpp|h.in|thrift|mm|qml|java|cs)$/) {
            print_if_verbose "[OK - BSD (.$1)] $filename\n";
            $ok++;
        } elsif ("$filename" =~ /\/compiler\..+\.in$/) {
            print_if_verbose "[OK - BSD (compiler.*.in)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - BSD (c++ style)] $filename\n";
            $errors++;
        }
        next;
    }

    # Script style BSD-3-Clause
    if ("$txt" =~ /$str_bsd3_script/s) {
        if ("$filename" =~ /\.(cmake|cmake\.in|cmake\.template|ps1|sh|py|pl|tcl|rb)$/) {
            print_if_verbose "[OK - BSD (.$1)] $filename\n";
            $ok++;
        } elsif ("$filename" =~ /CMakeLists.txt$/) {
            print_if_verbose "[OK - BSD (CMakeLists.txt)] $filename\n";
            $ok++;
        } elsif ("$filename" !~ /\./) {
            print_if_verbose "[OK - BSD (no extension)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - BSD (script)] $filename\n";
            $errors++;
        }
        next;
    }

    # Xml style BSD-3-Clause
    if ("$txt" =~ /$str_bsd3_xml/s) {
        if ("$filename" =~ /\.(xml|xml.template)$/) {
            print_if_verbose "[OK - BSD (.xml)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - BSD (xml style)] $filename\n";
            $errors++;
        }
        next;
    }

    # Lua style BSD-3-Clause
    if ("$txt" =~ /$str_bsd3_lua/s) {
        if ("$filename" =~ /\.lua$/) {
            print_if_verbose "[OK - BSD (.lua)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - BSD (lua style)] $filename\n";
            $errors++;
        }
        next;
    }

    # Swig style BSD-3-Clause
    if ("$txt" =~ /$str_bsd3_swig/s) {
        if ("$filename" =~ /\.(i)$/) {
            print_if_verbose "[OK - BSD (.$1)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - BSD (swig style)] $filename\n";
            $errors++;
        }
        next;
    }

    # Matlab style BSD-3-Clause
    if ("$txt" =~ /$str_bsd3_mat/s) {
        if ("$filename" =~ /\.(m)$/) {
            print_if_verbose "[OK - BSD (.$1)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - BSD (mat style)] $filename\n";
            $errors++;
        }
        next;
    }

    # C++ style LGPL2.1+
    if ("$txt" =~ /$str_lgpl2_1_or_later/s) {
        if ("$filename" =~ /src\/libYARP_([a-zA-Z]+)/) {
            print "[NOT OK - LGPL2.1+ (library $1)] $filename\n";
            $errors++;
        } elsif ("$filename" =~ /\.(cpp|h|thrift|qml)$/) {
            print_if_verbose "[OK - LGPL2.1+ (.$1)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - LGPL2.1+] $filename\n";
            $errors++;
        }
        next;
    }

    # Lua style LGPL2.1+ (other)
    if ("$txt" =~ /$str_lgpl2_1_or_later_lua/s) {
        if ("$filename" =~ /src\/libYARP_([a-zA-Z]+)/) {
            print "[NOT OK - LGPL2.1+ (library $1)] $filename\n";
            $errors++;
        } elsif ("$filename" =~ /\.(lua)$/) {
            print_if_verbose "[OK - LGPL2.1+ (.$1)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - LGPL2.1+] $filename\n";
            $errors++;
        }
        next;
    }

    # LGPL2.1+ (other)
    if ("$txt" =~ /$str_lgpl2_1_or_later_other/s ||
        "$txt" =~ /$str_lgpl2_1_or_later_lua_other/s) {
        print "[NOT OK - LGPL2.1+ (other)] $filename\n";
        $errors++;
        next;
    }

    # GPL2+
    if ("$txt" =~ /$str_gpl2_or_later/s) {
        if ("$filename" =~ /src\/libYARP_gsl\/.*\.(cpp|h)/ ||
            "$filename" =~ /example\/matrix\/.*\.(cpp|h)/) {
            # YARP_gsl library is necessarily GPL2+
            print_if_verbose "[OK - GPL2+ (.$1)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - GPL2+] $filename\n";
            $errors++;
        }
        next;
    } elsif ("$txt" =~ /$str_gpl2_or_later_other/s) {
        print "[NOT OK - GPL2+ (other)] $filename\n";
        $errors++;
        next;
    }

    # GPL3+
    if ("$txt" =~ /$str_gpl3_or_later/s) {
        if ("$filename" =~ /src\/guis\/yarpviz\/.*\.(cpp|h|qml)/ ||
            "$filename" =~ /src\/guis\/yarpscope\/.*\.(cpp|h|qml)/ ||
            "$filename" =~ /example\/ContainerExample\/.*\.(cpp|h|qml)/) {
            # yarpviz and yarpscope are necessarily GPL3+
            print_if_verbose "[OK - GPL3+ (.$1)] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - GPL3+] $filename\n";
            $errors++;
        }
        next;
    } elsif ("$txt" =~ /$str_gpl3_or_later_other/s) {
        print "[NOT OK - GPL3+ (other)] $filename\n";
        $errors++;
        next;
    }

    # Apache 2.0
    if ("$txt" =~ /$str_apache_2/s) {
        if ("$filename" eq "src/yarpidl_thrift/src/t_yarp_generator.cc") {
            print_if_verbose "[OK - Apache 2.0] $filename\n";
            $ok++;
        } else {
            print "[NOT OK - Apache 2.0] $filename\n";
            $errors++;
        }
        next;
    }

    print "[NOT OK (unknown license)] $filename\n";
    $errors++;
    next;
}

print_if_verbose "\n";
print_if_verbose "---\n";
print_if_verbose "FILES:  $files\n";
print_if_verbose "OK:     $ok\n";
print_if_verbose "SKIP:   $skip\n";
print_if_verbose "KNOWN (COUNTED):  $known\n";
print_if_verbose "KNOWN (DECLARED FROM FILE):  $number_files_marked_to_skip\n";
print_if_verbose "ERRORS: $errors\n";
print_if_verbose "---\n";
print_if_verbose "\n";

if ($ok + $skip + $known + $errors != $files) {
    print "[ERROR: Some file was not counted]\n\n";
    exit 1;
}

if ($known < $number_files_marked_to_skip) {
    print "[ERROR: Some known file was not found and the skip file was not updated]\n\n";
    exit 1;
}

if ($known > $number_files_marked_to_skip) {
    print "[ERROR: Some new known file was added and the skip file was not updated]\n\n";
    exit 1;
}

if ($errors != 0) {
    print "[ERROR: Some file has an invalid license]\n\n";
    exit 1;
}

exit 0;
